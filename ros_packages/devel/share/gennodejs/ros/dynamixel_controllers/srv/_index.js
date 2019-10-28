
"use strict";

let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetSpeed = require('./SetSpeed.js')
let RestartController = require('./RestartController.js')
let StartController = require('./StartController.js')
let StopController = require('./StopController.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let TorqueEnable = require('./TorqueEnable.js')

module.exports = {
  SetComplianceMargin: SetComplianceMargin,
  SetSpeed: SetSpeed,
  RestartController: RestartController,
  StartController: StartController,
  StopController: StopController,
  SetTorqueLimit: SetTorqueLimit,
  SetCompliancePunch: SetCompliancePunch,
  SetComplianceSlope: SetComplianceSlope,
  TorqueEnable: TorqueEnable,
};
