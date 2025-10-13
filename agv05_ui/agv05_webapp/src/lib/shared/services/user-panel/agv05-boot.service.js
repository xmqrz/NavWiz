import { elevatedPostAPI as postAPI } from '$lib/utils';

export const agv05Boot = {
  extendCountdown: function () {
    return postAPI('/boot', {
      operation: 10
    });
  },
  startRobot: function (mode = 1) {
    return postAPI('/boot', {
      operation: mode
    });
  },
  stopRobot: function () {
    return postAPI('/boot', {
      operation: 0
    });
  },
  softReboot: function () {
    return postAPI('/boot', {
      operation: -1
    });
  },
  hardReboot: function () {
    return postAPI('/boot', {
      operation: -2
    });
  },
  powerOff: function () {
    return postAPI('/boot', {
      operation: -3
    });
  },
  safeSoftReboot: function () {
    return postAPI('/boot', {
      operation: -4
    });
  },
  hotReload: function () {
    return postAPI('/boot', {
      operation: -5
    });
  }
};
