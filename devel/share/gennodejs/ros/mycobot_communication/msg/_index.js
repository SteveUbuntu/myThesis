
"use strict";

let MycobotPumpStatus = require('./MycobotPumpStatus.js');
let MycobotGripperStatus = require('./MycobotGripperStatus.js');
let MycobotSetEndType = require('./MycobotSetEndType.js');
let MycobotSetToolReference = require('./MycobotSetToolReference.js');
let MycobotSetAngles = require('./MycobotSetAngles.js');
let MycobotAngles = require('./MycobotAngles.js');
let MycobotSetFreshMode = require('./MycobotSetFreshMode.js');
let MycobotCoords = require('./MycobotCoords.js');
let MycobotSetCoords = require('./MycobotSetCoords.js');

module.exports = {
  MycobotPumpStatus: MycobotPumpStatus,
  MycobotGripperStatus: MycobotGripperStatus,
  MycobotSetEndType: MycobotSetEndType,
  MycobotSetToolReference: MycobotSetToolReference,
  MycobotSetAngles: MycobotSetAngles,
  MycobotAngles: MycobotAngles,
  MycobotSetFreshMode: MycobotSetFreshMode,
  MycobotCoords: MycobotCoords,
  MycobotSetCoords: MycobotSetCoords,
};
