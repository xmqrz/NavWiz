import EventEmitter from 'eventemitter3';
import SockJS from 'sockjs-client';
import { sockjsConnected } from 'stores/states';

export class Sock extends EventEmitter {
  constructor(url) {
    super();
    this.url = url;
    this.conn = null;
    this.timer = null;
    this.reconnectMultiplier = 1;
  }

  connect() {
    console.log('SockJS: Connecting...');
    this.disconnect();

    this.conn = new SockJS(this.url, null, {
      transports: 'websocket'
    });

    this.conn.onopen = () => {
      console.log('SockJS: Connected.');
      sockjsConnected.set(true);

      this.reconnectMultiplier = 1;
      this.emit('open');
      this.emit('connectionchange', true);
    };

    this.conn.onmessage = (event) => {
      if (this.timer) {
        clearInterval(this.timer);
        this.timer = setInterval(this.disconnect.bind(this), 10000);
      }
      try {
        this.handleMessage(JSON.parse(event.data));
      } catch (e) {
        return;
      }
    };

    this.conn.onclose = () => {
      console.log('SockJS: Disconnected.');
      sockjsConnected.set(false);

      if (this.conn === null) {
        return;
      }

      this.conn = null;

      setTimeout(
        this.connect.bind(this),
        (1500 + Math.random() * 60) * this.reconnectMultiplier
      );
      this.reconnectMultiplier = Math.min(this.reconnectMultiplier * 2, 20); // max period of 30 secs
      this.emit('close');
      this.emit('connectionchange', false);
    };
  }

  disconnect() {
    if (this.conn !== null) {
      console.log('SockJS: Disconnecting...');

      let conn = this.conn;
      this.conn = null;
      conn.close();
    }
  }

  handleMessage(msg) {
    if (msg && msg.channel && msg.data) {
      this.emit(msg.channel, msg.data);
    }
  }

  isConnected() {
    return this.conn && this.conn.readyState > 0;
  }

  send(msg) {
    if (!this.isConnected()) {
      return;
    }
    this.conn.send(JSON.stringify(msg));
  }

  on(ev, cb) {
    if (!this.isSubsExist()) {
      this.connect();
    }
    return super.on(ev, cb);
  }

  off(ev, cb) {
    // TODO: for nav between 2 page that use the port. this will disconnect? is hover preload solve this?
    let ret = super.off(ev, cb);
    if (!this.isSubsExist()) {
      this.disconnect();
    }
    return ret;
  }

  isSubsExist() {
    return Boolean(this.eventNames().length);
  }
}

export let sock = new Sock(SOCK_URL);
export let systemSock = new Sock(SYSTEM_SOCK_URL);
