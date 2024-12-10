
"use strict";

let GetRobotMode = require('./GetRobotMode.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let RawRequest = require('./RawRequest.js')
let GetProgramState = require('./GetProgramState.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let Popup = require('./Popup.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let Load = require('./Load.js')
let AddToLog = require('./AddToLog.js')

module.exports = {
  GetRobotMode: GetRobotMode,
  IsInRemoteControl: IsInRemoteControl,
  IsProgramRunning: IsProgramRunning,
  RawRequest: RawRequest,
  GetProgramState: GetProgramState,
  GetSafetyMode: GetSafetyMode,
  GetLoadedProgram: GetLoadedProgram,
  Popup: Popup,
  IsProgramSaved: IsProgramSaved,
  Load: Load,
  AddToLog: AddToLog,
};
