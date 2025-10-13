/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import Station from '../models/station';
import Junction from '../models/junction';

export default function (viz, scene, EditMode) {
  /* Objects under construction */
  var $viz = $(viz.node());
  // var canvas = viz.canvas;
  var Utils = scene.Utils;
  var construct = scene.append('g').attr('class', 'live-construct');
  var pt, theta;
  construct.pose = construct.append('use').attrs({
    class: 'station active',
    'xlink:href': '#station',
    display: 'none'
  });

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);

  function hovered(e) {
    /* jshint validthis: true */
    if (
      scene.mode >= EditMode.LIVE_SET_POSE &&
      scene.mode <= EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE
    ) {
      let ob = !e.altKey && snapToObject(this.mouse);
      if (ob instanceof Station) {
        viz.style('cursor', 'pointer');
        hoverObject.set(ob);
        drawPose();
      } else {
        viz.style('cursor', 'crosshair');
        hoverObject.set(null);
        let pt;
        if (ob instanceof Junction) {
          pt = [ob.x, ob.y];
        } else {
          pt = viz.zoom.pixelToCoord(this.mouse);
        }
        let theta = 0;
        drawPose(pt, theta);
      }
    }
  }

  function hoverended() {
    drawPose();
    hoverObject.set(null);
  }

  /* Track event */
  var track = d3
    .track(true)
    .on('trackstart', trackstarted)
    .on('track', tracked)
    .on('trackend', trackended);

  function trackstarted(e) {
    /* jshint validthis: true */
    if (
      scene.mode >= EditMode.LIVE_SET_POSE &&
      scene.mode <= EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE
    ) {
      let ob = !e.altKey && snapToObject(this.p0);
      if (ob instanceof Station) {
        scene.activeObject.set(ob);
        if (scene.mode === EditMode.LIVE_SET_POSE) {
          viz.liveApp.setStation();
        } else if (scene.mode === EditMode.LIVE_GOTO_POSE) {
          viz.liveApp.setGoTo();
        } else if (scene.mode === EditMode.LIVE_REVERSE_GOTO_POSE) {
          viz.liveApp.setGoTo(true);
        } else if (scene.mode === EditMode.LIVE_DYNAMIC_GOTO_POSE) {
          viz.liveApp.setGoTo(false, true);
        } else if (scene.mode === EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE) {
          viz.liveApp.setGoTo(true, true);
        }
        scene.activeObject.set(null);
        sendEditMode(EditMode.POINTER);
        return;
      }
      viz.style('cursor', 'crosshair');
      hoverObject.set(null);
      viz.call(noHover);
      if (ob instanceof Junction) {
        this._p0 = [ob.x, ob.y];
      } else {
        this._p0 = viz.zoom.pixelToCoord(this.p0);
      }
      pt = this._p0;
      theta = 0;
      drawPose(pt, theta);
    }
  }

  function tracked() {
    /* jshint validthis: true */
    if (
      scene.mode >= EditMode.LIVE_SET_POSE &&
      scene.mode <= EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE
    ) {
      pt = this._p0;
      if (this.p1) {
        this._p1 = viz.zoom.pixelToCoord(this.p1);
        theta = Utils.thetaTwoPoints(this._p0, this._p1);
      } else {
        theta = 0;
      }
      drawPose(pt, theta);
    }
  }

  function trackended() {
    /* jshint validthis: true */
    tracked.call(this);
    if (scene.mode === EditMode.LIVE_SET_POSE) {
      drawPose();
      viz.liveApp.setPose(
        {
          x: pt[0],
          y: pt[1],
          theta: theta
        },
        viz.models.ocgId()
      );
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.LIVE_GOTO_POSE) {
      drawPose();
      viz.liveApp.setGoToPose(
        {
          x: pt[0],
          y: pt[1],
          theta: theta
        },
        viz.models.ocgId()
      );
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.LIVE_REVERSE_GOTO_POSE) {
      drawPose();
      viz.liveApp.setGoToPose(
        {
          x: pt[0],
          y: pt[1],
          theta: theta
        },
        viz.models.ocgId(),
        true
      );
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.LIVE_DYNAMIC_GOTO_POSE) {
      drawPose();
      viz.liveApp.setGoToPose(
        {
          x: pt[0],
          y: pt[1],
          theta: theta
        },
        viz.models.ocgId(),
        false,
        true
      );
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE) {
      drawPose();
      viz.liveApp.setGoToPose(
        {
          x: pt[0],
          y: pt[1],
          theta: theta
        },
        viz.models.ocgId(),
        true,
        true
      );
      sendEditMode(EditMode.POINTER);
    }
  }

  function sendEditMode(m) {
    scene.mode = m;
    scene.setEditMode(m);
    $viz.trigger('scene.editMode');
  }

  function setEditMode(m) {
    if (m >= EditMode.LIVE_SET_POSE && m <= EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE) {
      viz.call(hover);
      viz.call(track);
      viz.style('cursor', 'default');
      viz.on('mouseenter.hover').call(viz.node()); // Hack to reenable hover events without mouse exiting svg first.
    } else {
      drawPose();
    }
  }

  function drawPose(pt, theta) {
    if (!pt) {
      construct.pose.attr('display', 'none');
      return;
    }
    construct.pose.attrs({
      transform: `translate(${pt[0]},${pt[1]})rotate(${(theta * 180.0) / Math.PI})`,
      display: 'block'
    });
  }

  $viz.on('live.updated', function () {
    if (scene.mode === EditMode.LIVE_SET_POSE && !viz.liveApp.canSetPose()) {
      drawPose();
      sendEditMode(EditMode.POINTER);
    } else if (
      scene.mode >= EditMode.LIVE_GOTO_POSE &&
      scene.mode <= EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE
    ) {
      if (!viz.liveApp.canGoToPose()) {
        drawPose();
        sendEditMode(EditMode.POINTER);
      }
    }
  });

  // Utils to make it compatible with ocg map.
  function snapToObject(pt) {
    if (!viz.models.selectObject) {
      return;
    }
    return Utils.snapToObject(pt);
  }

  var hoverObject = {
    set: function (ob) {
      if (scene.hoverObject) {
        scene.hoverObject.set(ob);
      }
    }
  };

  return {
    setEditMode: setEditMode
  };
}
