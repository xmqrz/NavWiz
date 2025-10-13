/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as d3 from 'd3';
import models from './models';
import sceneF from './scene';
import zoomF from '../../../routes/(config-panel)/config/task-templates/add/zoom';

export default function (svg, options) {
  /* Select svg object */
  var viz = d3.select(svg);
  viz.attr('class', 'viz');

  /* Options */
  viz.options = Object.assign(
    {
      editable: false,
      preserve: true
    },
    options
  );

  /* Metadata */
  viz.meta = {
    width: 0,
    height: 0,
    scale: 1,
    scaleExtent: [0.1, 2],
    translate: [0, 0]
  };
  viz.meta.scene = {
    x: -500,
    y: -500,
    width: 1000,
    height: 1000
  };

  /* Import svg definitions */
  //viz.defs = require('./agv05_webserver-task-template/defs')(viz);

  /* Layers: Scene */
  viz.scene = sceneF(viz);

  /* Zoom Updates */
  viz.zoomed = function () {
    viz.scene.zoomed();
  };

  /* Pan & Zoom */
  viz.zoom = zoomF(viz);
  viz.zoom.enable();

  /* Dimension Updates */
  viz.resized = function (width, height) {
    viz.meta.width = width;
    viz.meta.height = height;

    viz.zoom.resized();
    viz.scene.resized();
  };

  /* Model Updates */
  viz.modelsUpdated = function () {
    viz.scene.modelsUpdated();
    if (!viz.models.rawTaskTemplateName()) {
      viz.attr('hidden', 'true');
    } else {
      viz.attr('hidden', null);
    }
  };

  /* Utils */
  viz.trigger = function (eventName) {
    let event = new Event(eventName);
    viz.node().dispatchEvent(event);
  };
  viz.bind = function (eventName, callback) {
    viz.node().addEventListener(eventName, callback);
  };
  viz.unbind = function (eventName, callback) {
    viz.node().removeEventListener(eventName, callback);
  };

  viz.models = models(viz);

  return viz;
}
