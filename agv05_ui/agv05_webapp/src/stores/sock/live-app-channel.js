/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

'use strict';

import EventEmitter from 'eventemitter3';
import { liveAppChannel as _liveAppChannel } from './index';

export default (function (window) {
  var SubChannel = function (robotId) {
    this.robotId = robotId;
    this.subscribed = false;

    var that = this;
    this._messageCallback = function (submsg) {
      if (submsg && submsg.id && submsg.data && submsg.id === that.robotId) {
        that.emit('message', submsg.data);
      }
    };
  };

  SubChannel.prototype = new EventEmitter();

  SubChannel.prototype.subscribe = function (callback) {
    if (typeof callback === 'function') {
      this.on('message', callback);
    }

    if (this.subscribed) {
      return;
    }

    _liveAppChannel.subscribe(this._messageCallback);
    this.subscribed = true;
  };

  SubChannel.prototype.unsubscribe = function (callback) {
    if (callback) {
      this.off('message', callback);
      if (this.listenerCount('message')) {
        return;
      }
    }

    if (!this.subscribed) {
      return;
    }

    _liveAppChannel.unsubscribe(this._messageCallback);
    this.subscribed = false;
  };

  SubChannel.prototype.publish = function (data) {
    _liveAppChannel.publish({
      id: this.robotId,
      data: data
    });
  };

  var _subs = {};
  _liveAppChannel.getSub = function (robotId) {
    if (!(robotId in _subs)) {
      _subs[robotId] = new SubChannel(robotId);
    }
    return _subs[robotId];
  };

  return _liveAppChannel;
})(window);
