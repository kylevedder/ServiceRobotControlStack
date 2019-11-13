pf = {
  kLaserStdDev = 0.1;
  kArcStdDev = 0.1;
  kRotateStdDev = 0.04;
  kTemporalConsistencyWeight = 0;

  kMap = "./src/ServiceRobotControlStack/control_stack/maps/loop.map";
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
  kDesiredCommandX = 0.2;
  kDesiredCommandRot = 0;
  kOdomFilteringPriorBias = 0.7;
  kThresholdRotateInPlace = 0.9;
  kTranslateCommandSign = 1;
};

limits = {
  kMaxTraAcc = 3;
  kMaxTraVel = 1;
  kMaxRotAcc = 2;
  kMaxRotVel = 1;
};