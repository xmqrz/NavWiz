/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import vizF from 'mapx-layout-editor/viz';
import resizeHandleF from '$lib/shared/resize-handle';
import sceneF from './scene';
import modelsF from './models';

export default function (outer) {
  var map = $(outer);

  var viz = vizF(map, {
    grid: map.attr('grid') === 'true'
  });
  map.find('#id_media_url').after(viz.node());

  if (map.attr('resizable') === 'true') {
    resizeHandleF(viz.node());
  }

  // Extend viz.scene
  sceneF(viz);
  modelsF(viz);

  // Block default context menu
  map.on('contextmenu', function (event) {
    event.preventDefault();
  });

  $(window).on('resize', viz.resized);
  viz.resized();

  var $viz = $(viz.node());
  var formform = $viz.parent();
  var form = formform.parent();
  var mediaUrl = form[0].media_url.value;

  var qmap = form.find('#id_qmap').on('change', load);
  var statistic = form.find('#id_statistic').on('change', load);

  function load() {
    var base = mediaUrl + qmap.val();
    var suffix = '?t=' + Date.now();
    var models = {};
    var d1 = _loadLayout(base, suffix, models);
    var d2 = _loadOcg(base, suffix, models);

    // clear everything before reload
    viz.models.clearAll();
    viz.models.load({ ocg: -1 });
    viz.models.clearQmap();

    Promise.all([d1, d2])
      .then(() => {
        viz.models.load(models);
      })
      .catch((error) => {
        console.error('Error:', error);
      });

    d2.then(function (metadata) {
      _loadQmap(base, suffix, metadata);
    }).catch(function (error) {
      console.error('Error loading qmap:', error);
    });
  }

  async function _loadLayout(base, suffix, models) {
    try {
      const response = await fetch(`${base}_layout.json${suffix}`);
      if (!response.ok) {
        throw new Error('Failed to load map layout from URL.');
      }
      const data = await response.json();
      models.structure = data;
      return data;
    } catch (error) {
      console.error(error.message);
    }
  }

  async function _loadOcg(base, suffix, models) {
    try {
      const response = await fetch(`${base}_ocg.json${suffix}`);
      if (!response.ok) {
        throw new Error('Failed to load ocg metadata from URL.');
      }
      const data = await response.json();
      models.ocgChoices = [
        {
          id: 0,
          metadata: JSON.stringify(data),
          url: base + '_ocg.png' + suffix
        }
      ];
      return data;
    } catch (error) {
      console.error(error.message);
    }
  }

  function _loadQmap(base, suffix, metadata) {
    var png = {};
    var d1 = _loadQmapPng(base, suffix, 'avg', png);
    var d2 = _loadQmapPng(base, suffix, 'min', png);
    var d3 = _loadQmapPng(base, suffix, 'max', png);
    var d4 = _loadQmapPng(base, suffix, 'count', png);

    var stat = statistic.val();
    var url = base + '_' + stat + '.png' + suffix;

    Promise.all([d1, d2, d3, d4])
      .then(() => {
        viz.models.loadQmap({
          metadata: metadata,
          url: url,
          png: png
        });
      })
      .catch((error) => {
        console.error('Error:', error);
      });
  }

  function _loadQmapPng(base, suffix, stat, models) {
    return new Promise(function (resolve, reject) {
      var xhr = new XMLHttpRequest();
      xhr.open('GET', base + '_' + stat + '.png' + suffix);
      xhr.responseType = 'arraybuffer';

      xhr.onload = function () {
        if (xhr.status >= 200 && xhr.status < 300) {
          models[stat] = xhr.response;
          resolve();
        } else {
          console.error('Failed to load qmap from URL:', xhr.statusText);
          reject(xhr.statusText);
        }
      };
      xhr.onerror = function () {
        console.error('Failed to load qmap from URL.');
        reject('XHR Error');
      };

      xhr.send(null);
    });
  }

  load();

  return {
    destroy() {
      $(window).off('resize', viz.resized);
    }
  };
}
