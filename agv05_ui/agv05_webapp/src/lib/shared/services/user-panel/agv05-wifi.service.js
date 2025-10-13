import { postAPI, elevatedPostAPI } from '$lib/utils';

export const agv05WiFi = {
  ADHOC: function (state) {
    return elevatedPostAPI('/wifi/operation', {
      wifiop: 0,
      state: state
    });
  },
  WIFILIST: function () {
    return postAPI('/wifi/operation', {
      wifiop: 1,
      state: true
    });
  },
  WNICS: function (state) {
    return elevatedPostAPI('/wifi/operation', {
      wifiop: 2,
      state: state
    });
  },
  STATUS: function () {
    return elevatedPostAPI('/wifi/operation', {
      wifiop: 3,
      state: true
    });
  }
};
