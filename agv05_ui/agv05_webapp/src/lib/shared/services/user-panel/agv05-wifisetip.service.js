import { elevatedPostAPI as postAPI } from '$lib/utils';

export const agv05WiFiSetIP = {
  SETIP: function (staticmode, ip, netmask, gateway, dnsNameserver) {
    return postAPI('/wifi/setip', {
      staticmode: staticmode,
      ip: ip,
      netmask: netmask,
      gateway: gateway,
      dns_nameserver: dnsNameserver
    });
  }
};
