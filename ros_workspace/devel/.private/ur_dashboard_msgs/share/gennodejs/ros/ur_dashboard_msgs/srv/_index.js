
"use strict";

let GetSafetyMode = require('./GetSafetyMode.js')
let AddToLog = require('./AddToLog.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Load = require('./Load.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetRobotMode = require('./GetRobotMode.js')
let Popup = require('./Popup.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let RawRequest = require('./RawRequest.js')
let GetProgramState = require('./GetProgramState.js')

module.exports = {
  GetSafetyMode: GetSafetyMode,
  AddToLog: AddToLog,
  IsInRemoteControl: IsInRemoteControl,
  Load: Load,
  GetLoadedProgram: GetLoadedProgram,
  IsProgramSaved: IsProgramSaved,
  GetRobotMode: GetRobotMode,
  Popup: Popup,
  IsProgramRunning: IsProgramRunning,
  RawRequest: RawRequest,
  GetProgramState: GetProgramState,
};
