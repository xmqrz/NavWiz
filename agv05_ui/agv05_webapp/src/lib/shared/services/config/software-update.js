import { getAPI, postAPI } from '$lib/utils';

function successUpdateMsg() {
  return 'Update is now in progress. Please do not power off the PC during the update.';
}

function getUsb(fetch = undefined) {
  return getAPI('/config/software-update/usb-info', fetch, 'OPTIONS');
}

function getUsbVersion(usb, fetch = undefined) {
  return postAPI('/config/software-update/usb-info', { usb }, fetch);
}

function usbUpdate(usb, version, install_kiosk, kiosk, fetch = undefined) {
  return postAPI('/config/software-update/usb', { usb, version, install_kiosk }, fetch);
}

export default {
  successUpdateMsg,

  getUsb,
  getUsbVersion,
  usbUpdate
};
