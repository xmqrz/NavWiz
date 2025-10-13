import { sock, systemSock } from './sock';
import { Channel as BaseChannel, channelReadable as baseChannelReadable } from './channel';
import { StatusAPIChannel } from './status-channel';

export { sock, systemSock };
// NOTE: Channel must subscribe first before can publish. (Due to sock auto connect impl).

export class Channel extends BaseChannel {
  constructor(name) {
    super(sock, name);
  }
}

export class SystemChannel extends BaseChannel {
  constructor(name) {
    super(systemSock, name);
  }
}

export function channelReadable(name) {
  return baseChannelReadable(sock, name);
}

export function systemChannelReadable(name) {
  return baseChannelReadable(systemSock, name);
}

export const moduleChannel = new Channel('module');
export const cameraChannel = new Channel('camera');
export const amclChannel = new Channel('amcl');
export const mapXChannel = new Channel('mapx');
export const mapChannel = new Channel('map');
export const notificationChannel = new Channel('notification');
export const fmsChannel = new Channel('fms');
export const popupChannel = new Channel('popup');
export const ioChannel = new Channel('io');
export const remoteAssistChannel = new Channel('remote-assist');
export const remoteIoChannel = new Channel('remote-io');
export const statusChannel = new Channel('status');
export const appLogChannel = new Channel('log');
export const statusAPIChannel = new StatusAPIChannel(statusChannel);

export const laserChannel = new SystemChannel('laser');
export const liveAppChannel = new SystemChannel('live-app');
export const logChannel = new SystemChannel('log');
export const softwareUpdateChannel = new SystemChannel('software-update');
export const validationDataChannel = new SystemChannel('validation-data');
