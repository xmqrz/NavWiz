import EventEmitter from 'eventemitter3';
import { getModalStore } from '@skeletonlabs/skeleton';

import { agv05Module } from '$lib/shared/services/user-panel/agv05-module.service.js';
import { httpErrorModal } from '$lib/modal-service';
import { moduleChannel } from 'stores/sock';

export const moduleService = function () {
  const modalStore = getModalStore();

  class SubChannel extends EventEmitter {
    constructor(moduleId) {
      super();
      this.moduleId = moduleId;
      this.subscribed = false;

      this._messageCallback = (submsg) => {
        if (submsg && submsg.id && submsg.data && submsg.id === this.moduleId) {
          this.emit('message', submsg.data);
        }
      };
    }

    subscribe(callback) {
      if (typeof callback === 'function') {
        this.on('message', callback);
      }

      if (this.subscribed) {
        return;
      }

      moduleChannel.subscribe(this._messageCallback);
      this.subscribed = true;
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

      moduleChannel.unsubscribe(this._messageCallback);
      this.subscribed = false;
    }

    publish(data) {
      moduleChannel.publish({
        id: this.moduleId,
        data: data
      });
    }

    startModule() {
      _manager.startModule(this.moduleId);
    }

    stopModule() {
      _manager.stopModule(this.moduleId);
    }
  }

  var _subs = {};
  function getSub(moduleId) {
    if (!(moduleId in _subs)) {
      _subs[moduleId] = new SubChannel(moduleId);
    }
    return _subs[moduleId];
  }

  var _manager = getSub('__');
  _manager.startModule = function (moduleId) {
    var title;
    if (moduleId === 'task-runner') {
      title = 'Failed to start task runner.';
    } else {
      title = 'Failed to start app.';
    }
    agv05Module.startModule(moduleId).catch(function (http) {
      modalStore.trigger(httpErrorModal(title, http));
      // send empty start_module callback to clear loading icon
      _manager.emit('message', {
        command: 'start_module'
      });
    });
  };
  _manager.stopModule = function (moduleId) {
    var title;
    if (moduleId === 'task-runner') {
      title = 'Failed to stop task runner.';
    } else {
      title = 'Failed to stop app.';
    }
    agv05Module.stopModule(moduleId).catch((e) => {
      console.error(e);
      modalStore.trigger(httpErrorModal(title));
    });
  };
  _manager.getHwApp = async function (moduleId) {
    let data = await agv05Module.getModule(moduleId);
    data = data || {};
    if (moduleId) {
      return data;
    }
    return data['hw-app-descriptions'] || [];
  };

  function getManager() {
    return _manager;
  }

  function spin() {}

  function stop() {}

  return {
    spin,
    stop,
    getSub,
    getManager
  };
};
