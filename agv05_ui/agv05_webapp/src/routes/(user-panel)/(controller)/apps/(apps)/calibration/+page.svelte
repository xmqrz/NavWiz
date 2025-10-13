<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';
  import { onMount, getContext } from 'svelte';

  import AppLayout from 'components/AppLayout.svelte';
  import { alertModal } from '$lib/modal-service.js';
  import { defaultdict } from '$lib/utils';

  const mainController = getContext('mainController');
  const calibration = mainController.moduleService.getSub('calibration');
  const modalStore = getModalStore();

  let yaw = new Yaw();
  let profile = 1;
  let ahead = 0.5;
  let status = null;
  let running = false;
  let runningMotor = false;
  let runningDone = false;
  let runningTunePID = false;
  let data = defaultdict({}, 0.0);

  function calibrateBattery() {
    calibration.publish({
      command: 'calibrate_battery'
    });
  }

  function calibrateLine() {
    calibration.publish({
      command: 'calibrate_line'
    });
  }

  function calibrateLinePID(profile, ahead, dist) {
    calibration.publish({
      command: 'calibrate_line_pid',
      profile: profile,
      ahead: ahead,
      distance: dist
    });
  }

  function calibrateMotorWheel(dist) {
    calibration.publish({
      command: 'calibrate_motor',
      type: 'wheel',
      distance: dist
    });
  }

  function calibrateSave() {
    calibration.publish({
      command: 'save',
      data: data
    });
  }

  function calibrateCommand(cmd) {
    calibration.publish({
      command: cmd
    });
  }

  /* Yaw class */
  const MAX_SPEED = 0.2;
  const SPEED_INC = 0.01;
  const SPEED_DEC = 0.05;

  function Yaw() {
    this.yawSpeed = 0.0;
    this._driving = null;
    this._offset = null;
  }
  Yaw.prototype.left = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      that.yawSpeed = Math.min(MAX_SPEED, that.yawSpeed + SPEED_INC);
      that._publish();
    }, 100);
  };
  Yaw.prototype.right = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      that.yawSpeed = Math.max(-MAX_SPEED, that.yawSpeed - SPEED_INC);
      that._publish();
    }, 100);
  };
  Yaw.prototype.release = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      if (that.yawSpeed > 0) {
        that.yawSpeed = Math.max(0, that.yawSpeed - SPEED_DEC);
      } else if (that.yawSpeed < 0) {
        that.yawSpeed = Math.min(0, that.yawSpeed + SPEED_DEC);
      } else {
        that._cancelDriving();
      }
      that._publish();
    }, 100);
  };
  Yaw.prototype.stop = function () {
    this._cancelDriving();
    this.yawSpeed = 0;
    this._publish();
  };
  Yaw.prototype._cancelDriving = function () {
    if (this._driving) {
      clearInterval(this._driving);
      this._driving = null;
    }
  };
  Yaw.prototype._publish = function () {
    calibration.publish({
      command: 'calibrate_motor',
      type: 'yaw',
      speed: this.yawSpeed
    });
  };

  function calibrationCallback(_data) {
    if (_data.command === 'status') {
      status = _data.status;
      running = _data.status !== 'idle';
      runningMotor = _data.status.indexOf('motor') !== -1;
      runningDone = _data.status.indexOf('done') !== -1;
      runningTunePID = _data.status.indexOf('pid') !== -1;
    } else if (_data.command === 'error') {
      modalStore.trigger(alertModal('Calibration Error', _data.error));
    } else if (_data.command === 'outcome') {
      modalStore.trigger(alertModal('Calibration Outcome', _data.outcome));
    } else if (_data.command === 'data') {
      if (_data.data.left_distance !== undefined) {
        _data.data.left_distance = _data.data.left_distance.toFixed(3);
      }
      if (_data.data.right_distance !== undefined) {
        _data.data.right_distance = _data.data.right_distance.toFixed(3);
      }
      if (_data.data.wheel_distance !== undefined) {
        _data.data.wheel_distance = _data.data.wheel_distance.toFixed(3);
      }
      if (_data.data.left_front_distance !== undefined) {
        _data.data.left_front_distance = _data.data.left_front_distance.toFixed(3);
      }
      if (_data.data.left_rear_distance !== undefined) {
        _data.data.left_rear_distance = _data.data.left_rear_distance.toFixed(3);
      }
      if (_data.data.right_front_distance !== undefined) {
        _data.data.right_front_distance = _data.data.right_front_distance.toFixed(3);
      }
      if (_data.data.right_rear_distance !== undefined) {
        _data.data.right_rear_distance = _data.data.right_rear_distance.toFixed(3);
      }
      if (_data.data.yaw_angle !== undefined) {
        _data.data.yaw_angle = _data.data.yaw_angle.toFixed(1);
      }
      data = JSON.parse(JSON.stringify(_data.data));
    }
  }

  onMount(() => {
    calibration.subscribe(calibrationCallback);
    //allow time for subscribe to happen fisrt.
    setTimeout(function () {
      calibration.publish({
        command: 'status'
      });
    }, 100);

    return () => {
      calibration.unsubscribe(calibrationCallback);
    };
  });

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module <br/> "Calibration" ?',
      response: (r) => {
        if (r) {
          calibration.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }

  const calibrationTitle = 'font-bold px-8 py-4';
</script>

<AppLayout title="Calibration" moduleID="calibration" on:close={shutdown}>
  <div class="space-y-4 p-4">
    <div class="card shadow-lg">
      <div class={calibrationTitle}>Battery Calibration</div>
      <hr />
      <div class="px-8 py-4">
        <button
          type="button"
          class="variant-filled-warning btn"
          on:click={calibrateBattery}
          disabled={running}>
          <i class="fa-solid fa-play"></i>Start
        </button>
        {#if status === 'calibrate_battery'}
          <span>&nbsp; Calibrating battery...</span>
        {/if}
      </div>
    </div>
    <div class="card shadow-lg">
      <div class={calibrationTitle}>Line Sensor Calibration</div>
      <hr />
      <div class="px-8 py-4">
        <button
          type="button"
          class="variant-filled-warning btn"
          on:click={calibrateLine}
          disabled={running}>
          <i class="fa-solid fa-play"></i> Start
        </button>
        {#if status === 'calibrate_line'}
          <button
            type="button"
            class="variant-filled-warning btn"
            on:click={() => calibrateCommand('cancel')}>
            <i class="fa-solid fa-eject"></i> Cancel
          </button>
        {/if}
        {#if status === 'calibrate_line'}
          <span>&nbsp; Calibrating line sensor...</span>
        {/if}
      </div>
    </div>
    <div class="card shadow-lg">
      <div class={calibrationTitle}>Motor Calibration</div>
      <hr />
      <div class="space-y-4 px-8 py-4">
        {#if !runningMotor}
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={running}
            on:click={() => calibrateMotorWheel(1)}>
            <i class="fa-solid fa-play"></i> Forward 1m
          </button>
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={running}
            on:click={() => calibrateMotorWheel(2)}>
            <i class="fa-solid fa-play"></i> Forward 2m
          </button>
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={running}
            on:click={() => calibrateMotorWheel(-1)}>
            <i class="fa-solid fa-play"></i> Reverse 1m
          </button>
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={running}
            on:click={() => calibrateMotorWheel(-2)}>
            <i class="fa-solid fa-play"></i> Reverse 2m
          </button>
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={running}
            on:click={() => yaw.stop()}>
            <i class="fa-solid fa-play"></i> Rotation
          </button>
        {/if}
        {#if runningMotor}
          <button
            type="button"
            class="variant-filled-warning btn"
            on:click={() => calibrateCommand('cancel')}>
            <i class="fa-solid fa-eject"></i> Cancel
          </button>
        {/if}
        {#if runningMotor && !runningDone}
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={status == null || status.indexOf('wheel') !== -1}
            on:click={() => calibrateCommand('done')}>
            <i class="fa-solid fa-stop"></i> Done
          </button>
        {/if}
        {#if runningMotor && runningDone}
          <button type="button" class="variant-filled-warning btn" on:click={calibrateSave}>
            <i class="fa-solid fa-floppy-disk"></i> Save
          </button>
        {/if}
        {#if status && status.indexOf('wheel') !== -1}
          <span>&nbsp; Rotation must be done after Forward/Reverse.</span>
        {/if}
        {#if status && status.indexOf('yaw') !== -1}
          <span>&nbsp; Forward/Reverse must be done before Rotation.</span>
        {/if}
        {#if runningMotor}
          {#if status && status.indexOf('wheel_diff') !== -1}
            <div>
              <span>Left Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.left_distance}
                disabled={!runningDone} />
            </div>
            <div>
              <span>Right Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.right_distance}
                disabled={!runningDone} />
            </div>
          {/if}
          {#if status && status.indexOf('wheel_tri') !== -1}
            <div>
              <span>Wheel Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.wheel_distance}
                disabled={!runningDone} />
            </div>
          {/if}
          {#if status && status.indexOf('wheel_meca') !== -1}
            <div>
              <span>Left Front Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.left_front_distance}
                disabled={!runningDone} />
            </div>
            <div>
              <span>Right Front Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.right_front_distance}
                disabled={!runningDone} />
            </div>
            <div>
              <span>Left Rear Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.left_rear_distance}
                disabled={!runningDone} />
            </div>
            <div>
              <span>Right Rear Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.right_rear_distance}
                disabled={!runningDone} />
            </div>
          {/if}
          {#if status && (status.indexOf('yaw_diff') !== -1 || status.indexOf('yaw_meca') !== -1)}
            <div>
              <span>Yaw Angle (°)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.1"
                disabled={!runningDone}
                bind:value={data.yaw_angle} />
            </div>
          {/if}
          {#if status && status.indexOf('yaw_tri') !== -1}
            <div>
              <span>Yaw Angle (°)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.1"
                disabled={!runningDone}
                bind:value={data.yaw_angle} />
            </div>
            <div>
              <span>Drift Distance (m)</span>
              <input
                class="keyboard input"
                type="number"
                step="0.001"
                bind:value={data.wheel_distance}
                disabled={!runningDone} />
            </div>
          {/if}
          {#if !runningDone && status && status.indexOf('yaw') !== -1}
            <div class="flex justify-center space-x-4">
              <button
                type="button"
                class="variant-filled-warning btn-icon px-10"
                on:touchstart={() => yaw.left()}
                on:touchend={() => yaw.release()}
                on:touchcancel={() => yaw.release()}
                on:mousedown={() => yaw.left()}
                on:mouseup={() => yaw.release()}>
                <i class="fa-solid fa-rotate-left fa-2x"></i>
              </button>
              <button
                type="button"
                class="variant-filled-error btn-icon px-10"
                on:touchstart={() => yaw.stop()}
                on:mousedown={() => yaw.stop()}>
                <i class="fa-regular fa-hand fa-2x"></i>
              </button>
              <button
                type="button"
                class="variant-filled-warning btn-icon px-10"
                on:touchstart={() => yaw.right()}
                on:touchend={() => yaw.release()}
                on:touchcancel={() => yaw.release()}
                on:mousedown={() => yaw.right()}
                on:mouseup={() => yaw.release()}>
                <i class="fa-solid fa-rotate-right fa-2x"></i>
              </button>
            </div>
          {/if}
        {/if}
      </div>
    </div>
    <div class="card shadow-lg">
      <div class={calibrationTitle}>Navigation PID Tuning</div>
      <hr />
      <div class="space-y-4 px-8 py-4">
        {#if !runningTunePID}
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={running}
            on:click={() => calibrateLinePID(profile, ahead, 3)}>
            <i class="fa-solid fa-play"></i> Forward 3m
          </button>
          <button
            type="button"
            class="variant-filled-warning btn"
            disabled={running}
            on:click={() => calibrateLinePID(profile, ahead, -3)}>
            <i class="fa-solid fa-play"></i> Reverse 3m
          </button>
        {/if}
        {#if runningTunePID}
          <button
            type="button"
            class="variant-filled-warning btn"
            on:click={() => calibrateCommand('cancel')}>
            <i class="fa-solid fa-eject"></i> Cancel
          </button>
        {/if}
        {#if runningTunePID}
          <span>&nbsp; Tuning PID Profile {profile}...</span>
        {/if}
        {#if !runningTunePID}
          <div>
            <span>Profile</span>
            <input
              class="keyboard input"
              type="number"
              bind:value={profile}
              min="1"
              max="5"
              step="1" />
          </div>
          <div>
            <span>Ahead Distance (m)</span>
            <input
              class="keyboard input"
              type="number"
              step="0.001"
              bind:value={ahead}
              min="0" />
          </div>
        {/if}
      </div>
    </div>
  </div>
</AppLayout>

<style>
  .btn > i {
    padding-right: 4px;
  }
</style>
