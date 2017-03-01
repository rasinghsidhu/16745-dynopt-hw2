---
# You don't need to edit this file, it's empty on purpose.
# Edit theme's home layout instead if you wanna make some changes
# See: https://jekyllrb.com/docs/themes/#overriding-theme-defaults
layout: page
title: Writeup
---

[Download Source Code Here](https://github.com/rasinghsidhu/fluffy-palm-tree/archive/v1.tar.gz)

## Part 1

![cmaes-10-iter](assets/cmaes-10-iter.gif)
Figure 1. Optimize walking under perfect conditions with 10 generations of CMAES (i.e. no noise and no perturbations)

Figure 1 shows an example of an optimized controller with no noise or perturbations. To generate it I used cmaes-c implementation and optimized the objective function of the controller. It doesn't change much from the initial parameters. Similarly running it for more iterations does not change the gait significantly.

## Part 2

![cmaes-noise](assets/cmaes-20-iter-noise.gif)
Figure 2. White noise of 15Nm added to torque commands on joints.

In figure 2, white noise was added to the torque commands on the joints. Initially the controller failed and would fall over but after a few more iterations (roughly 20) the controller was able to deal with the noise.

![cmaes-wind](assets/cmaes-100-iter-perturb-wind.gif)
Figure 3. Constant wind of 5N added to torso.

In figure 3, the controller was trained to deal with a constant 5N "wind" added to the torso. You can see that it clearly outpaces the top 2 controllers while maintaining a very similar gait by using the additional energy.

![standing-failure](assets/standing_failure.gif)
Figure 4. Constant wind added, but no tweak of objective function.

However, in this situation it became very difficult to train the controller, especially with the objective function as it was. In order to get an effective controller I had to lower weights on force and torque limits scores and raise the initial std deviations on my parameters. This let CMAES search a wider space for a better solution.


I tried to optimize the controller to resist pushes of 5-100N forces of .4 seconds with a .0025% chance of occurring like in [Optimizing Walking Controllers for Uncertain Inputs and Environments.](http://www.dgp.toronto.edu/~jmwang/optuie/) However I had a hard time training with the simulator as there are a few bugs. There were some states that would achieve a very low cost that really corresponded to the simulator sliding along the ground (like in figure 4) or just falling over continuously. In order to prevent this I knew I would need to tweak the objective function to penalize this behavior but did not have the time to dig into all the simulator code in order to add ideas like penalizing controllers that didnt take more than 5 proper steps, and possibly penalizing large std deviations in step duration. 

I managed to get a controller at 5N, but anything larger and the optimizer kept falling into valleys where the controller would slide along the ground. I tried various things like tuning parameters individually and tweaking weights on existing objective parameters but still ran into the issue with the controller falling into valleys of slipping along the ground.

![5N-Push](assets/cmaes-100-iter-5N-Push.gif)
Figure 5. 5N pushes in either +,- direction lasting .4 seconds with a probability of occurring at .0025%. 

