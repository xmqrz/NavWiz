import { getModalStore } from '@skeletonlabs/skeleton';
import { getContext } from 'svelte';
import { get, writable, derived } from 'svelte/store';

import { notificationChannel } from 'stores/sock';

export function downtimeTrackerService() {
  const modalStore = getModalStore();
  const user = getContext('user');

  var _hasTracker;
  var tracker = writable(undefined);
  var trackerText;
  var trackerPopup;

  var closeTracker = function () {
    if (trackerPopup) {
      modalStore.close(trackerPopup);
      trackerPopup = null;
    }
  };

  var hasTracker = derived([user, tracker], ([$user, $tracker], set) => {
    if ($tracker === 'robot_controller') {
      set($user.has_perms('app.start_agv05') || $user.has_perms_with_pin('app.start_agv05'));
      return;
    } else if ($tracker === 'task_runner') {
      set(
        $user.has_perms('app.start_task_runner') ||
          $user.has_perms_with_pin('app.start_task_runner')
      );
      return;
    }
    set(false);
  });

  function showTracker() {
    var r = get(tracker);
    var t = trackerText;
    if (!r) {
      return;
    }
    closeTracker();

    trackerPopup = {
      type: 'component',
      component: 'modalDowntimeTracker',
      position: 'items-center',
      title: 'Downtime Activity Tracker',
      subTitle: `Why did the ${r.replace('_', ' ')} stop ${t} ?`,
      response: (res) => {
        trackerPopup = null;
        if (!res) {
          return;
        }
        notificationChannel.publish({
          id: 'tracker',
          tracker: r,
          text: t,
          activity: res
        });
      }
    };
    modalStore.trigger(trackerPopup);
  }

  function notifyCb(data) {
    if (data.id === 'tracker') {
      tracker.set(data.tracker);
      trackerText = data.text;
      _hasTracker = get(hasTracker);
      if (_hasTracker) {
        showTracker();
      } else {
        closeTracker();
      }
    }
  }

  function spin() {
    notificationChannel.subscribe(notifyCb);
  }

  function stop() {
    notificationChannel.unsubscribe(notifyCb);
  }

  return {
    hasTracker: () => hasTracker,
    showTracker: showTracker,
    spin: spin,
    stop: stop
  };
}
