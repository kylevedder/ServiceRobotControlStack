pf = {
  kLaserStdDev = 0.1;
  kArcStdDev = 0.1;
  kRotateStdDev = 0.04;
  kTemporalConsistencyWeight = 0;

  kMap = "/home/k/code/catkin_ws/src/ServiceRobotControlStack/control_stack/maps/loop.map";
  kInitX = 4;
  kInitY = 0;
  kInitTheta = 0;
  kRobotRadius = 0.1;
  kCollisionRollout = 2;
};

od = {
  kMinDistanceThreshold = 0.05;
  kProposedTranslationStdDev = 1.0;
  kProposedRotationStdDev = 5;
};