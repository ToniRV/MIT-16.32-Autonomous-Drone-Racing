# Project for MIT 16.32 Principles of Optimal Control and Estimation

## TODO
- [ ] Write paper:
    - [ ] Abstract:
    - [ ] Intro:
    - [ ] Sota:
    - [ ] Methodology:
    - [ ] Results:
    - [ ] Conclusion:=
- Read papers about drone dynamics
- Formulate shortest time optimization problem mathematically
  - Write optimization cost: integral of dt (minimum time problem)
  - Write constraints for each phase (automate constraint writting for phases)
  - Write boundary conditions
- Parse gates in flight goggles
- Implement problem in GPOPS
- Solve optimal control problem for only one phase
- Solve optimal control problem for two phases
- Iterate for all gates/phases.
- Close the loop of the trajectory, final and start positions should match +- epsilon.
- Extract static Tfs from optimal trajectory with timestamps.
- Bundle these tfs in a rosbag?
- Use trajPlayback to playback the trajectories
- Sit back and write down all you did.

## References

- Drone dynamics:
  - Double Integrator: https://en.wikipedia.org/wiki/Double_integrator
  (eso es double integrator, pero es muy fácil extenderlo a triple integrator, 4th integrator,...)
  - Las lineales las saque, parte de aqui
    - http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
    - https://www.sciencedirect.com/science/article/pii/S1877705814031981
- Simulators: 

 
