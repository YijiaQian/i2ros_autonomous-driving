
"use strict";

let ProcessStatus = require('./ProcessStatus.js');
let NodeList = require('./NodeList.js');
let FlightEvent = require('./FlightEvent.js');
let WaypointList = require('./WaypointList.js');
let TelemString = require('./TelemString.js');
let FlightStateTransition = require('./FlightStateTransition.js');
let ImageDetections = require('./ImageDetections.js');
let Keypoint = require('./Keypoint.js');
let NodeStatus = require('./NodeStatus.js');
let ControlMessage = require('./ControlMessage.js');
let Box = require('./Box.js');
let Detection = require('./Detection.js');
let Latency = require('./Latency.js');
let FlightCommand = require('./FlightCommand.js');
let ImageSegmentation = require('./ImageSegmentation.js');
let JoyDef = require('./JoyDef.js');

module.exports = {
  ProcessStatus: ProcessStatus,
  NodeList: NodeList,
  FlightEvent: FlightEvent,
  WaypointList: WaypointList,
  TelemString: TelemString,
  FlightStateTransition: FlightStateTransition,
  ImageDetections: ImageDetections,
  Keypoint: Keypoint,
  NodeStatus: NodeStatus,
  ControlMessage: ControlMessage,
  Box: Box,
  Detection: Detection,
  Latency: Latency,
  FlightCommand: FlightCommand,
  ImageSegmentation: ImageSegmentation,
  JoyDef: JoyDef,
};
