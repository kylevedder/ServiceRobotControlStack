pf = {
  kLaserStdDev = 0.051;
  kArcStdDev = 0.051;
  kRotateStdDev = 0.021;
  kTemporalConsistencyWeight = 0;

  kMap = "/home/k/code/catkin_ws/src/ParticleFilterCpp/particle_filter/maps/loop.map";
  kInitX = 0;
  kInitY = 0;
  kInitTheta = 0;
  kRobotRadius = 0.2;
  kCollisionRollout = 2;
};

od {
  kProposedTranslationStdDev = 1.0;
  kProposedRotationStdDev = 0.9;
};