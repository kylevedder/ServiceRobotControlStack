-- Copyright 2019 kvedder@seas.upenn.edu
-- School of Engineering and Applied Sciences,
-- University of Pennsylvania
--
-- This software is free: you can redistribute it and/or modify
-- it under the terms of the GNU Lesser General Public License Version 3,
-- as published by the Free Software Foundation.
--
-- This software is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU Lesser General Public License for more details.
--
-- You should have received a copy of the GNU Lesser General Public License
-- Version 3 in the file COPYING that came with this distribution.
-- If not, see <http://www.gnu.org/licenses/>.
-- ========================================================================

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
  kSafetyMargin = 0.1;
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