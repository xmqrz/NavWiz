import { elevatedPostAPI as postAPI } from '$lib/utils';

export const agv05WiFiCon = {
  CONNECT: function (sid, user, pass, security, eap, hidden = false) {
    return postAPI('/wifi/connect', {
      op: 0,
      ssid: sid,
      identity: user,
      password: pass,
      enc: security,
      eap: eap,
      hidden: hidden
    });
  },
  FORGET: function (sid) {
    return postAPI('/wifi/connect', {
      op: 1,
      ssid: sid,
      password: ''
    });
  }
};
