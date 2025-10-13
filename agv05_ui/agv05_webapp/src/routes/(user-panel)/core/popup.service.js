import { getModalStore } from '@skeletonlabs/skeleton';
import { getContext } from 'svelte';
import { get } from 'svelte/store';
import dompurify from 'dompurify';

import { popupChannel } from 'stores/sock';
import { countdownTimer, popupType } from 'stores/states.js';

var popupTitle = {
  'non-interactive': 'Wait',
  alert: 'Message',
  confirm: 'Confirm',
  keypad: 'Input',
  'safety-triggered': 'Safety Triggered',
  'safety-resume': 'Safety Resume',
  'safety-resuming': 'Resuming'
};

export const popupService = function () {
  const modalStore = getModalStore();
  const user = getContext('user');

  var alarm = new Audio();
  alarm.src = window.agvPanelToken ? '' : '/static/system/audio/alarm.wav';
  alarm.loop = true;

  var muteNotification = false;
  var popup;
  var countdown;

  function popupCb(data) {
    popupType.set('');
    let displayCountdown = false;
    if (countdown) {
      clearInterval(countdown);
      countdown = null;
    }

    if (popup) {
      modalStore.close(popup);
      popup = undefined;
      alarm.pause();
    }

    let $user = get(user);

    if (!(data.type in popupTitle)) {
      return;
    } else if (data.type.indexOf('safety') === 0) {
      if (!$user.has_perms('app.show_popup_safety')) {
        return;
      }
      popupType.set('safety');
    } else {
      // extract mentions of user with '@'.
      data.users = [];
      let words = data.message.split(' ').reverse();
      while (words.length) {
        let w = words.pop();
        if (!w.length) {
          continue;
        }
        if (w[0] === '@') {
          data.users.push(w.substr(1));
        } else {
          words.push(w);
          break;
        }
      }

      if (!data.users.length) {
        if (!$user.has_perms('app.show_popup_user_public')) {
          return;
        }
      } else if (!$user.has_perms('app.show_popup_user_private')) {
        if (!$user.has_perms('app.show_popup_user_own')) {
          return;
        }
        if (data.users.indexOf(user.username) < 0) {
          return;
        }
        // Todo: handle user_with_pin
      }
      data.message = words.reverse().join(' ');
      popupType.set('user');
    }

    // don't open modal if muted
    if (muteNotification) {
      return;
    }

    data.message = data.message.replace(/\n/g, '<br/>');
    data.message = dompurify.sanitize(data.message);

    if (data.timer > 0) {
      displayCountdown = true;
      countdownTimer.set(data.timer);
      countdown = setInterval(function () {
        countdownTimer.update((t) => {
          if (t > 0) {
            t -= 1;
          } else {
            clearInterval(countdown);
            countdown = null;
          }
          return t;
        });
      }, 1000);
    }

    let title = popupTitle[data.type];
    if (data.users && data.users.length) {
      if (data.users.length > 1) {
        title += ` (@+${data.users.length} users)`;
      } else {
        title += ` (@${data.users[0]})`;
      }
    }

    popup = {
      type: 'component',
      component: 'modalPopup',
      position: 'items-center',
      title,
      displayCountdown,
      message: data.message,
      msgType: data.type,
      response: (r) => {
        popup = undefined;
        alarm.pause();
        if (r === 'mute') {
          muteNotification = true;
          return;
        }
        if (r === undefined) {
          return;
        }
        popupChannel.publish({
          type: data.type,
          value: r
        });
      }
    };
    modalStore.priorityTrigger(popup);

    if (!data.muted) {
      let p = alarm.play();
      // may catch error due to no user interaction.
      if (p) {
        p.catch(() => {});
      }
    }
  }

  function retrieve() {
    muteNotification = false;
    popupChannel.publish({
      type: 'retrieve_all'
    });
  }

  function spin() {
    popupChannel.subscribe(popupCb);
    setTimeout(retrieve, 100);
  }

  function stop() {
    popupChannel.unsubscribe(popupCb);
    alarm.pause();
  }

  return {
    retrieve: retrieve,
    spin: spin,
    stop: stop,
    subscribe: popupChannel.subscribe.bind(popupChannel),
    unsubscribe: popupChannel.unsubscribe.bind(popupChannel)
  };
};
