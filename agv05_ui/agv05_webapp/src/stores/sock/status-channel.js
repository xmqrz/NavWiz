import EventEmitter from 'eventemitter3';
import * as _ from 'lodash-es';

export class StatusAPIChannel extends EventEmitter {
  constructor(channel) {
    super();
    this.statusSet = new Set();
    this.funcMap = new Map();
    this.channel = channel;
    this.subscribed = false;
    this.diagnostics = {};
    this.__messageCallback = this._messageCallback.bind(this);
  }

  _messageCallback(data) {
    if (!this.subscribed || data.id !== 'diagnostics') {
      return;
    }
    this.diagnostics = _.mergeWith(this.diagnostics, data.diagnostics, merger);
    for (const status of this.statusSet) {
      if (status === '*') {
        this.emit(status, this.diagnostics);
        continue;
      }
      let keys = status.split(':');
      let children = this.diagnostics.status.children;
      let values = [];
      let message = '';
      for (const key of keys) {
        for (const c of children) {
          if (c.name === key) {
            children = c.children;
            values = c.values;
            message = c.message;
            break;
          }
        }
      }
      this.emit(status, {
        values,
        message
      });
    }

    function merger(objValue, srcValue, key) {
      if (key === 'children' && Array.isArray(objValue) && Array.isArray(srcValue)) {
        let objMap = {};
        for (let v of objValue) {
          if (v.name) {
            objMap[v.name] = v;
          }
        }

        let res = [];
        for (let v of srcValue) {
          if (v.name) {
            res.push(_.mergeWith(objMap[v.name], v, merger));
          }
        }

        res.collapsed = objValue.collapsed;
        return res;
      }
    }
  }

  subscribe(callback, status) {
    if (!status || typeof callback !== 'function') {
      return;
    }
    if (this.funcMap.has(callback)) {
      return;
    }

    this.on(status, callback);
    this.statusSet.add(status);
    this.funcMap.set(callback, status);
    if (!this.subscribed) {
      this.subscribed = true;
      this.channel.subscribe(this.__messageCallback);
    }
  }

  unsubscribe(callback) {
    if (!callback || !this.funcMap.has(callback)) {
      return;
    }
    const status = this.funcMap.get(callback);

    this.off(status, callback);
    this.funcMap.delete(callback);

    if (!this.listenerCount(status)) {
      this.statusSet.delete(status);
    }

    if (this.subscribed && this.statusSet.size === 0) {
      this.subscribed = false;
      this.channel.unsubscribe(this.__messageCallback);
    }
  }
}
