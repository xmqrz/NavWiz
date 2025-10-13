/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
/* bottom panel template */
var bottomPanelHtml = `
<div class="agv05x-toolbar-bottom">
  <button type="button" class="btn variant-filled-tertiary flex-col live-deadman-switch" tip-title="Press and hold to execute live app command.">
    Execute Live App Cmd <br/>
    <span class="text-xs">(Shift + Spacebar)</span>
  </button>
</div>
`;

export default function (viz) {
  var $viz = $(viz.node());
  var deadmanPressed = false;
  let heartbeatInterval;

  var bottomPanel = $(bottomPanelHtml);
  var liveDeadmanSwitch = bottomPanel.find('.live-deadman-switch');
  liveDeadmanSwitch.show = () => {
    liveDeadmanSwitch.css('display', 'flex');
  };
  liveDeadmanSwitch.hide();
  liveDeadmanSwitch
    .on('mousedown', function () {
      if (!deadmanPressed) {
        deadmanPressed = true;
        onDeadmanChange(deadmanPressed);
      }
    })
    .on('mouseup mouseleave', function () {
      if (deadmanPressed) {
        deadmanPressed = false;
        onDeadmanChange(deadmanPressed);
      }
    });

  function updateLiveApp() {
    var isLiveConnected = viz.liveApp.connected();

    liveDeadmanSwitch.hide();

    if (!isLiveConnected) {
      return;
    }

    if (viz.liveApp.canSendNavToCmd()) {
      liveDeadmanSwitch.show();
    }
  }

  $viz.on('live.updated', updateLiveApp);

  function onDeadmanChange(state) {
    if (state) {
      liveDeadmanSwitch.addClass('pressed');
      if (!heartbeatInterval) {
        heartbeatInterval = setInterval(heartbeat, 1000);
      }
    } else {
      liveDeadmanSwitch.removeClass('pressed');
      if (heartbeatInterval) {
        clearInterval(heartbeatInterval);
        heartbeatInterval = null;
      }
    }
  }

  function heartbeat() {
    viz.liveApp.sendNavToCmd();
  }

  // Fullscreen
  $viz.on('showFullscreen', function () {
    bottomPanel.addClass('agv05-fullscreen');
  });
  $viz.on('hideFullscreen', function () {
    bottomPanel.removeClass('agv05-fullscreen');
  });

  function onKeyup(event) {
    if (deadmanPressed) {
      if (event.shiftKey || event.code === 'Space') {
        deadmanPressed = false;
        onDeadmanChange(deadmanPressed);
        event.preventDefault();
      }
    }
  }

  function onKeydown(event) {
    if (event.shiftKey && event.code === 'Space') {
      // prevent default scrolling behaviors
      event.preventDefault();
    }
    if (!deadmanPressed) {
      if (event.shiftKey && event.code === 'Space') {
        deadmanPressed = true;
        onDeadmanChange(deadmanPressed);
        event.preventDefault();
      }
    }
  }

  bottomPanel.insertBefore($viz);

  function disable() {
    liveDeadmanSwitch.attr('disabled', 'disabled');
  }

  function enable() {
    liveDeadmanSwitch.removeAttr('disabled');
  }

  function destroy() {
    $(document.body).off('keyup', onKeyup).off('keydown', onKeydown);
  }

  $(document.body).on('keyup', onKeyup).on('keydown', onKeydown);

  return {
    disable: disable,
    enable: enable,
    destroy: destroy
  };
}
