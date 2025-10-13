/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

var PADDING = 20;
var XOFFSET = 5;

export default function (viz, scene) {
  var $scene = $(scene.node());
  var Utils = scene.Utils;
  var popupData;
  var popupHidden = false;

  var note = scene.overlay.append('g');
  note.attr('class', 'note');
  var toggleBtn = note.append('g');
  var popup = note.append('g');

  function modelsUpdated() {
    let ao = scene.activeObject.get();

    let data = [];
    let actions = viz.models.rawActions();
    popupData = undefined;
    $scene.find('.node').each((i, elm) => {
      if (!elm.id.startsWith('node')) {
        return;
      }
      let action_id = parseInt(elm.id.slice('4')) - 1;
      if (action_id <= 0 || action_id >= actions.length) {
        return;
      }

      let action = actions[action_id];
      if (!action.tooltip) {
        return;
      }

      let bbox = elm.getBBox();

      if (ao.action_id === action_id) {
        popupData = {
          action_id: action_id,
          action: action,
          bbox: bbox
        };
      }

      data.push({
        action_id: action_id,
        action: action,
        bbox: bbox
      });
    });

    toggleBtn
      .selectAll('text')
      .data(data)
      .join('text')
      .text('\uF044')
      .attr('class', 'fa')
      .attr('font-weight', 'normal')
      .attr('font-size', 14)
      .attr('text-anchor', 'left')
      .attr('dominant-baseline', 'middle')
      .attr('x', (d) => d.bbox.x + d.bbox.width - 37)
      .attr('y', (d) => d.bbox.y + 60)
      .on('click', function (event, d) {
        let ao = scene.activeObject.get();

        // check toggle popup
        if (popupData && popupData.action_id === d.action_id) {
          popupHidden = !popupHidden;
        }

        // check change active object
        let redraw = !ao.outcome_key && ao.action_id === d.action_id;
        scene.activeObject.set(d.action_id, null);
        if (redraw) {
          popupData = d;
          drawPopup();
        }

        event.preventDefault();
      });

    drawPopup();
  }

  function drawPopup() {
    popup.select('g').remove();

    if (popupHidden || !popupData) {
      return;
    }

    let lines = popupData.action.tooltip.split('\n');
    let textWidth = 0;
    lines.forEach((l) => {
      let w = Utils.textWidth(l);
      if (w > textWidth) {
        textWidth = w;
      }
    });
    let width = textWidth + PADDING * 2;
    let height = lines.length * 16 + PADDING * 2;
    let x = popupData.bbox.x + popupData.bbox.width + XOFFSET;
    let y = popupData.bbox.y;
    let textX = x + PADDING;
    let baseTextY = y + PADDING + 7;

    let g = popup.append('g');
    let bg = g.append('rect'); // note background
    bg.attr('fill', '#dfa7df')
      .attr('rx', 15)
      .attr('width', width)
      .attr('height', height)
      .attr('x', x)
      .attr('y', y);
    bg.style('filter', 'drop-shadow(2px 4px 6px black)');
    lines.forEach((l, i) => {
      let textY = baseTextY + i * 16;
      g.append('text') // note text
        .text(l)
        .attr('font-weight', 'normal')
        .attr('font-size', 14)
        .attr('font-family', 'Times,serif')
        .attr('text-anchor', 'left')
        .attr('dominant-baseline', 'middle')
        .attr('x', textX)
        .attr('y', textY);
    });
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
