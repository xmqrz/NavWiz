import $ from 'cash-dom';
import base64 from 'base64-js';

import { laserChannel } from 'stores/sock';
import vizF from './viz';
import errorF from './err';
import modelsF from './models';
import tableF from './table';

const CONFIG_PREFIX = 'lidar_';

const layoutHtml = `
<div class="agv05-laser-sensors">
  <div class="top-div w-full"></div>
  <div class="grid grid-cols-[3fr_1fr] space-x-3">
    <div class="left-div"></div>
    <div class="right-div space-y-3"></div>
  </div>
</div>
`;

export default function (outer, data) {
  var editor = $(outer);

  var layout = $(layoutHtml);
  var leftDiv = layout.find('.left-div');
  var rightDiv = layout.find('.right-div');
  var topDiv = layout.find('.top-div');
  editor.prepend(layout);

  var viz = vizF(editor);
  // TODO: put on left layout
  var $viz = $(viz.node());

  leftDiv.append(viz.node());

  errorF(viz, topDiv);
  modelsF(viz);
  tableF(viz, rightDiv);

  function load() {
    let laser_topics,
      lidar_1_topics,
      lidar_2_topics,
      lidar_3_topics,
      lidar_4_topics,
      lidar_5_topics;
    let configs, defaults;

    // Read data
    try {
      laser_topics = JSON.parse(data.lasers.laser_topics);
    } catch (e) {
      console.log('Laser area laser_parameters parse error.');
    }

    try {
      lidar_1_topics = !Array.isArray(laser_topics.lidar_1_topics)
        ? []
        : laser_topics.lidar_1_topics;
    } catch (e) {
      console.log('Laser area lidar_1_topics parse error.');
      lidar_1_topics = [];
    }

    try {
      lidar_2_topics = !Array.isArray(laser_topics.lidar_2_topics)
        ? []
        : laser_topics.lidar_2_topics;
    } catch (e) {
      console.log('Laser area lidar_2_topics parse error.');
      lidar_2_topics = [];
    }

    try {
      lidar_3_topics = !Array.isArray(laser_topics.lidar_3_topics)
        ? []
        : laser_topics.lidar_3_topics;
    } catch (e) {
      console.log('Laser area lidar_3_topics parse error.');
      lidar_3_topics = [];
    }

    try {
      lidar_4_topics = !Array.isArray(laser_topics.lidar_4_topics)
        ? []
        : laser_topics.lidar_4_topics;
    } catch (e) {
      console.log('Laser area lidar_4_topics parse error.');
      lidar_4_topics = [];
    }

    try {
      lidar_5_topics = !Array.isArray(laser_topics.lidar_5_topics)
        ? []
        : laser_topics.lidar_5_topics;
    } catch (e) {
      console.log('Laser area lidar_5_topics parse error.');
      lidar_5_topics = [];
    }

    try {
      defaults = Object.keys(data.properties)
        .filter((key) => key.startsWith(CONFIG_PREFIX))
        .reduce((acc, key) => {
          let p = data.properties[key];
          if (p.initial === undefined || p.initial === null || p.initial === '') {
            return acc;
          }
          acc[key] = p.initial;
          return acc;
        }, {});
      configs = Object.keys(data.lasers)
        .filter((key) => key in defaults)
        .reduce((acc, key) => {
          acc[key] = data.lasers[key];
          return acc;
        }, {});
    } catch (e) {
      console.log('Laser area parameters parse error.');
    }

    viz.models.load({
      lidar_1_topics: lidar_1_topics,
      lidar_2_topics: lidar_2_topics,
      lidar_3_topics: lidar_3_topics,
      lidar_4_topics: lidar_4_topics,
      lidar_5_topics: lidar_5_topics,
      defaults: defaults,
      configs: configs
    });
  }

  function save() {
    editor.trigger('save');
  }

  function stage() {
    return viz.models.getParameters();
  }

  // Prevent leaving dirty forms
  $viz.on('models.dirty', function () {
    editor.trigger('dirty');
  });

  // Block default context menu
  editor.on('contextmenu', function (event) {
    event.preventDefault();
  });

  viz.editor = {
    save: save,
    stage: stage
  };

  load();
  $viz.trigger('models.ready');
  outer.viz = viz;
  editor.trigger('ready');
  $(window).on('resize', viz.resized);
  viz.resized();

  laserChannel.subscribe(laserCallback);

  // allow time for subscribe to happen first.
  window.setTimeout(function () {
    laserChannel.publish({
      id: 'retrieve_all'
    });
  }, 500);

  function laserCallback(data) {
    if (data.id === 'robot_svg') {
      viz.scene.updateRobotSvg(data.robot_svg);
    } else if (data.id === 'laser_pose') {
      viz.scene.updateLaserPose(data.laser_pose);
      viz.table.updateLaserPose(data.laser_pose);
    } else if (data.id === 'laser_scan') {
      data.laser_scan.ranges = new Float32Array(
        base64.toByteArray(data.laser_scan.ranges).buffer
      );
      if (data.laser_scan.intensities) {
        data.laser_scan.intensities = new Float32Array(
          base64.toByteArray(data.laser_scan.intensities).buffer
        );
      }
      viz.scene.updateLaserScan(data.laser_scan);
      viz.table.updateLaserScan(data.laser_scan);
    }
  }

  return {
    destroy() {
      $(window).off('resize', viz.resized);
      laserChannel.unsubscribe(laserCallback);
    }
  };
}
