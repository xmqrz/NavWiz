import EventEmitter from 'eventemitter3';
import { writable, readonly } from 'svelte/store';

export class Channel extends EventEmitter {
  constructor(sock, name) {
    super();
    this.sock = sock;
    this.name = name;
    this.subscribed = false;
    this.connected = false;

    this._messageCallback = (data) => {
      if (this.subscribed) {
        this.emit('message', data);
      }
    };
  }

  subscribe(callback) {
    if (typeof callback === 'function') {
      this.on('message', callback);
    }

    if (!this.connected) {
      this._subscribe = () => {
        if (this.subscribed) {
          this.sock.send({
            action: 'subscribe',
            channel: this.name
          });
        }
      };
      this.sock.on(this.name, this._messageCallback);
      this.sock.on('open', this._subscribe);
      this.connected = true;
    }

    if (!this.subscribed) {
      if (this.sock.isConnected()) {
        setTimeout(this._subscribe);
      }
      this.subscribed = true;
    }
  }

  unsubscribe(callback) {
    if (callback) {
      this.off('message', callback);
      if (this.listenerCount('message')) {
        return;
      }
    }

    if (!this.subscribed) {
      return;
    }

    this.sock.send({
      action: 'unsubscribe',
      channel: this.name
    });
    this.subscribed = false;

    // HACK: delay disconnect to avoid socket reconnect.
    // TODO: should we just maintain single socket that always connect...?
    setTimeout(() => {
      if (this.listenerCount('message')) {
        return;
      }

      if (!this.connected) {
        return;
      }

      this.sock.off(this.name, this._messageCallback);
      this.sock.off('open', this._subscribe);
      this.connected = false;
    }, 100);
  }

  publish(data) {
    if (!this.sock.isConnected()) {
      return;
    }
    this.sock.send({
      action: 'publish',
      channel: this.name,
      data: data
    });
  }
}

export function channelReadable(sock, name) {
  const channel = new Channel(sock, name);

  const channelW = writable({}, function (set) {
    function callback(data) {
      set(data);
    }
    channel.subscribe(callback);
    return function () {
      channel.unsubscribe(callback);
    };
  });

  return readonly(channelW);
}
