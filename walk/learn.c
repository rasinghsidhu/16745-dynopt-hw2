/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "main2.h"
#include "cmaes_interface.h"

/*****************************************************************************/
/*****************************************************************************/

extern SIM sim;

/*****************************************************************************/
/*****************************************************************************/
// Example of optimizing a value
/*****************************************************************************/
/*****************************************************************************/

double run_sim( SIM *sim )
{
  int i;

  for( i = 0; sim->time < sim->duration; i++ )
    {
      controller( sim );
      save_data( sim );
      if ( sim->status == CRASHED )
	       break;
      integrate_one_time_step( sim );
    }

  // write_the_mrdplot_file( sim );
  return get_score( sim );
}


/**

  s->swing_time = 0.89;

  s->thrust1 = -0.047;

  // knot point for swing hip
  s->swing_hip_target = 0.33;
  s->swing_hv1 = -0.37;
  s->swing_ha1 = -0.10;

  // first knot point for swing knee
  s->swing_knee1 = -0.43;
  s->swing_kv1 = 0.29;
  s->swing_ka1 = 0.26;

  // second knot point for swing knee
  s->swing_knee_target = -0.080;
  s->swing_kv2 = 0.11;
  s->swing_ka2 = -0.14;

  // one knot point for stance hip
  s->stance_hip_target = -0.22;
  s->stance_hv1 = -0.031;
  s->stance_ha1 = 0.28;

  s->pitch_d = 0.085;

  // first knot point for stance knee
  s->stance_kv1 = 0.41;
  s->stance_ka1 = -0.38;

  // second knot point for stance knee
  s->stance_knee_target = -0.046;
  s->stance_kv2 = -0.29;
  s->stance_ka2 = -0.36;

  s->stance_ankle_torque = -0.12; // set on transitions to X_SWING
**/

void setupParameters(SIM *s, double * params){
  s->swing_time = params[0];
  s->thrust1 = params[1];
  // knot point for swing hip
  s->swing_hip_target = params[2];
  s->swing_hv1 = params[3];
  s->swing_ha1 = params[4];
  // first knot point for swing knee
  s->swing_knee1 = params[5];
  s->swing_kv1 = params[6];
  s->swing_ka1 = params[7];
  // second knot point for swing knee
  s->swing_knee_target = params[8];
  s->swing_kv2 = params[9];
  s->swing_ka2 = params[10];
  // one knot point for stance hip
  s->stance_hip_target = params[11];
  s->stance_hv1 = params[12];
  s->stance_ha1 = params[13];

  s->pitch_d = params[14];

  // first knot point for stance knee
  s->stance_kv1 = params[15];
  s->stance_ka1 = params[16];

  // second knot point for stance knee
  s->stance_knee_target = params[17];
  s->stance_kv2 = params[18];
  s->stance_ka2 = params[19];

  s->stance_ankle_torque = params[20]; // set on transitions to X_SWING

  s->pos_gains[L_HIP][L_HIP] = params[21];
  s->vel_gains[L_HIP][L_HIP] = params[22];
  s->int_gains[L_HIP][L_HIP] = 0.0;
  s->pos_gains[L_KNEE][L_KNEE] = params[23];
  s->vel_gains[L_KNEE][L_KNEE] = params[24];
  s->int_gains[L_KNEE][L_KNEE] = 0.0;
  s->pos_gains[L_ANKLE][L_ANKLE] = 0.0;
  s->vel_gains[L_ANKLE][L_ANKLE] = 0.0;
  s->int_gains[L_ANKLE][L_ANKLE] = 0.0;
  s->pos_gains[R_HIP][R_HIP] = params[21];
  s->vel_gains[R_HIP][R_HIP] = params[22];
  s->int_gains[R_HIP][R_HIP] = 0.0;
  s->pos_gains[R_KNEE][R_KNEE] = params[23];
  s->vel_gains[R_KNEE][R_KNEE] = params[24];
  s->int_gains[R_KNEE][R_KNEE] = 0.0;
  s->pos_gains[R_ANKLE][R_ANKLE] = 0.0;
  s->vel_gains[R_ANKLE][R_ANKLE] = 0.0;
  s->int_gains[R_ANKLE][R_ANKLE] = 0.0;

  s->l1_duration = params[25];
  s->l1_lhip_target = params[26];
  s->l1_lknee_target = params[27];
  s->l1_rankle_torque = params[28]; // see LAUNCH1 action

  // Launch 2 parameters: see transition out of WAITING
  s->l2_duration = params[29];
  s->l2_lhip_target = s->l1_lhip_target;
  s->l2_lknee_target = params[30];
  s->l2_rhip_target = params[31];
  s->l2_rknee_target = params[32];
}
/*****************************************************************************/

/*
struct sim_t{
  double* ret_addr;
  double* params;
};

void threaded_sim()
*/

main( int argc, char **argv )
{
  int i;
  PARAMETER *params;
  int n_parameters;
  double score, new_score;

  init_default_parameters( &sim );
  sim.rand_scale = 0;
  sim.controller_print = 1;

  /* Parameter file argument? */
  if ( argc > 1 )
    {
      params = read_parameter_file( argv[1] );
      n_parameters = process_parameters( params, &sim, 1 );
      if ( n_parameters > MAX_N_PARAMETERS )
	{
	  fprintf( stderr, "Too many parameters %d > %d\n",
		   n_parameters, MAX_N_PARAMETERS );
	  exit( -1 );
	}
    }

  init_sim( &sim );
  init_data( &sim );

  sim.controller_print = 0;

  cmaes_t evo;
  double *arFunvals, *const* pop, *xfinal;

  arFunvals = cmaes_init(&evo, 0, NULL, NULL, 0, 0, "cmaes_initials.par");
  printf("%s\n", cmaes_SayHello(&evo));
  cmaes_ReadSignals(&evo, "cmaes_signals.par");
  while(!cmaes_TestForTermination(&evo))
      { 
        /* generate lambda new search points, sample population */
        pop = cmaes_SamplePopulation(&evo); /* do not change content of pop */

        /* evaluate the new search points using fitfun */
        for (i = 0; i < cmaes_Get(&evo, "lambda"); ++i) {
          setupParameters(&sim, pop[i]);
          arFunvals[i] = run_sim(&sim);
          reinit_sim(&sim);
        }

        /* update the search distribution used for cmaes_SamplePopulation() */
        cmaes_UpdateDistribution(&evo, arFunvals);  

        /* read instructions for printing output or changing termination conditions */ 
        cmaes_ReadSignals(&evo, "cmaes_signals.par");
        fflush(stdout); /* useful in MinGW */
    }
  printf("Stop:\n%s\n",  cmaes_TestForTermination(&evo)); /* print termination reason */
  cmaes_WriteToFile(&evo, "all", "allcmaes.dat");         /* write final results */

  /* get best estimator for the optimum, xmean */
  xfinal = cmaes_GetNew(&evo, "xbestever"); /* "xbestever" might be used as well */
  cmaes_exit(&evo); /* release memory */ 

  setupParameters(&sim, xfinal);
  arFunvals[i] = run_sim(&sim);
  write_the_mrdplot_file(&sim);

  free(xfinal); 
}

/*****************************************************************************/
