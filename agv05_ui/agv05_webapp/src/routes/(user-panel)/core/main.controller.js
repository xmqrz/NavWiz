import { popupService } from './popup.service.js';
import { moduleService } from './module.service.js';
import { downtimeTrackerService } from './downtime-tracker.service.js';
import { notificationChannel } from 'stores/sock';
import { robotRunning, robotName } from 'stores/states.js';
import { updateUserInfo } from 'stores/auth.js';

// TODO: move all states to here?? mainController.robotName.
export class MainController {
  constructor() {
    robotRunning.set(-1);
    this.popupService = popupService();
    this.moduleService = moduleService();
    this.downtimeTrackerService = downtimeTrackerService();
  }

  notifyCb(data) {
    if (data.id === 'robot_running') {
      robotRunning.set(data.value);
    } else if (data.id === 'robot_name') {
      robotName.set(data.value);
    } else if (data.id === 'auth') {
      delete data.id;
      // TODO: is it neccessary to update user?
      updateUserInfo(data);
    }
  }

  spin() {
    notificationChannel.subscribe(this.notifyCb);
    this.popupService.spin();
    this.moduleService.spin();
    this.downtimeTrackerService.spin();
  }

  stop() {
    notificationChannel.unsubscribe(this.notifyCb);
    this.popupService.stop();
    this.moduleService.stop();
    this.downtimeTrackerService.stop();
    robotRunning.set(-1);
  }
}
