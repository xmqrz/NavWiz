<script>
  import { onMount, getContext } from 'svelte';
  import { getModalStore, TabGroup, Tab } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import AppLayout from 'components/AppLayout.svelte';

  const mainController = getContext('mainController');
  const hardwareTest = mainController.moduleService.getSub('hardware-test');
  const modalStore = getModalStore();

  const obstacleOptions = [
    [0, 'Disable'],
    [1, 'Forward'],
    [2, 'Reverse'],
    [3, 'Rotate Left'],
    [4, 'Rotate Right'],
    [5, 'Forward End'],
    [6, 'Reverse End'],
    [7, 'Manual Control'],
    [8, 'Dynamic Forward'],
    [9, 'Dynamic Reverse'],
    [10, 'Forward Far'],
    [11, 'Reverse Far'],
    [12, 'Forward Middle'],
    [13, 'Reverse Middle'],
    [14, 'Forward Near'],
    [15, 'Reverse Near'],
    [16, 'Omni'],
    [17, 'Omni End'],
    [18, 'Omni Far'],
    [19, 'Omni Middle'],
    [20, 'Omni Near']
  ];

  let tab = 1;
  let selectedObstacleSensorProfile = 0;
  let selectedObstacleSensorArea = 0;
  let tabIo = {
    port: 1
  };
  let io = new Io();
  let led = new Led();
  let lineSensor = new DataSink();
  let motor = new Motor();
  let obstacleSensor = new ObstacleSensor();
  let panel = new Panel();
  let power = new DataSink();
  let rfid = new DataSink();
  let safety = new DataSink();
  let yaw = new Yaw();

  /* DataSink class */
  function DataSink() {}
  DataSink.prototype.update = function (data) {
    this.data = data;
  };

  function updateLineSensor(data) {
    lineSensor.data = data;
  }

  function updatePower(data) {
    power.data = data;
  }

  function updateRfid(data) {
    rfid.data = data;
  }

  function updateSafety(data) {
    safety.data = data;
  }

  function Io() {
    this._inputs = [0, 0, 0, 0, 0, 0, 0, 0];
    this._outputs = [0, 0, 0, 0, 0, 0, 0, 0];
    this._ioName = null;

    this.getInput = (port, pin) => this._inputs[port - 1] & (1 << pin);
    this.getOutput = (port, pin) => this._outputs[port - 1] & (1 << pin);
  }
  Io.prototype.toggleOutput = function (port, pin) {
    this._outputs[port - 1] ^= 1 << pin;
    hardwareTest.publish({
      id: 'io',
      data: {
        command: 'output',
        port: port,
        output: this._outputs[port - 1]
      }
    });
  };
  Io.prototype.getOutputName = function (port, pin) {
    if (!this._ioName || !this._ioName.output_name) {
      return '';
    }
    return this._ioName.output_name[port - 1][pin];
  };
  Io.prototype.getInputName = function (port, pin) {
    if (!this._ioName || !this._ioName.input_name) {
      return '';
    }
    return this._ioName.input_name[port - 1][pin];
  };
  function updateIo(data) {
    if (!data) {
      return;
    }
    if (!io._ioName && data.io_name) {
      io._ioName = data.io_name;
    }
    if (Array.isArray(data.inputs) && data.inputs.length === 8) {
      io._inputs = data.inputs;
    }
    if (Array.isArray(data.outputs) && data.outputs.length === 8) {
      io._outputs = data.outputs;
    }
  }

  /* Led class */
  function Led() {
    this.mode = 0;
  }
  Led.prototype.setMode = function (mode) {
    led.mode = mode;
    hardwareTest.publish({
      id: 'led',
      data: {
        command: 'mode',
        mode: mode
      }
    });
  };

  /* Motor class */
  function Motor() {
    this.leftSpeed = 0.0;
    this.rightSpeed = 0.0;
    this._driving = null;
  }
  const MAX_SPEED = 0.2;
  const SPEED_INC = 0.01;
  const SPEED_DEC = 0.05;

  Motor.prototype.leftForward = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      that.leftSpeed = Math.min(MAX_SPEED, that.leftSpeed + SPEED_INC);
      that._publish();
    }, 100);
  };
  Motor.prototype.leftReverse = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      that.leftSpeed = Math.max(-MAX_SPEED, that.leftSpeed - SPEED_INC);
      that._publish();
    }, 100);
  };
  Motor.prototype.leftRelease = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      if (that.leftSpeed > 0) {
        that.leftSpeed = Math.max(0, that.leftSpeed - SPEED_DEC);
      } else if (that.leftSpeed < 0) {
        that.leftSpeed = Math.min(0, that.leftSpeed + SPEED_DEC);
      } else {
        that._cancelDriving();
      }
      that._publish();
    }, 100);
  };
  Motor.prototype.rightForward = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      that.rightSpeed = Math.min(MAX_SPEED, that.rightSpeed + SPEED_INC);
      that._publish();
    }, 100);
  };
  Motor.prototype.rightReverse = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      that.rightSpeed = Math.max(-MAX_SPEED, that.rightSpeed - SPEED_INC);
      that._publish();
    }, 100);
  };
  Motor.prototype.rightRelease = function () {
    let that = this;
    this._cancelDriving();
    this._driving = setInterval(function () {
      if (that.rightSpeed > 0) {
        that.rightSpeed = Math.max(0, that.rightSpeed - SPEED_DEC);
      } else if (that.rightSpeed < 0) {
        that.rightSpeed = Math.min(0, that.rightSpeed + SPEED_DEC);
      } else {
        that._cancelDriving();
      }
      that._publish();
    }, 100);
  };
  Motor.prototype.stop = function () {
    this._cancelDriving();
    this.leftSpeed = 0;
    this.rightSpeed = 0;
    this._publish();
  };
  Motor.prototype._cancelDriving = function () {
    if (this._driving) {
      clearInterval(this._driving);
      this._driving = null;
    }
  };
  Motor.prototype._publish = function () {
    hardwareTest.publish({
      id: 'motor',
      data: {
        command: 'speed',
        left_speed: this.leftSpeed,
        right_speed: this.rightSpeed
      }
    });
  };
  function updateMotor(data) {
    motor.data = data;
  }

  /* ObstacleSensor class */
  function ObstacleSensor() {
    this.profile = 0;
    this.area = 0;
  }
  ObstacleSensor.prototype.setProfile = function (profile) {
    this.profile = profile;
    if (profile !== 0 && this.area > 20) {
      this.area = 0;
    }
    this._publish();
  };
  ObstacleSensor.prototype.setArea = function (area) {
    if (typeof area === 'string') {
      area = Number(area);
    }
    this.area = area;
    this._publish();
  };
  ObstacleSensor.prototype._publish = function () {
    hardwareTest.publish({
      id: 'laser',
      data: {
        command: 'area',
        profile: this.profile,
        area: this.area
      }
    });
  };
  function updateObstacleSensor(data) {
    obstacleSensor.data = data;
  }

  /* Panel class */
  function Panel() {
    this.leds = {};
  }
  Panel.prototype.toggleLed = function (led_name) {
    this.leds[led_name] = !this.leds[led_name];
    hardwareTest.publish({
      id: 'panel',
      data: {
        command: 'led_' + led_name,
        state: this.leds[led_name]
      }
    });
  };
  function updatePanel(data) {
    panel.data = data;
  }

  /* Yaw class */
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
    hardwareTest.publish({
      id: 'yaw',
      data: {
        command: 'speed',
        yaw_speed: this.yawSpeed
      }
    });
  };
  Yaw.prototype._rad2deg = function (rad) {
    rad %= 2 * Math.PI;
    rad += 2 * Math.PI;
    rad %= 2 * Math.PI;
    if (rad > Math.PI) {
      rad -= 2 * Math.PI;
    }
    return (rad * 180.0) / Math.PI;
  };
  function updateYaw(data) {
    const _key = (key) => '_' + key.toLowerCase().replace(/[^a-z0-9]/g, '_');
    if (!yaw._offset) {
      yaw._offset = {};
      data.forEach((pair) => {
        yaw._offset[_key(pair[0])] = parseFloat(pair[1]);
      });
      // this.data = JSON.parse(JSON.stringify(data));
      yaw.data = [];
    }
    yaw.data.length = data.length;
    data.forEach((pair, index) => {
      yaw.data[index] = [pair[0], yaw._rad2deg(pair[1] - yaw._offset[_key(pair[0])])];
    });
  }

  // helper function
  function formatLineSensorRaw(data) {
    if (!(Array.isArray(data) && data.length)) {
      return '-';
    }
    let n = data.length;
    let divider = (n & 3) === 0 ? n >> 2 : (n & 1) === 0 ? n >> 1 : n;

    let display = '';
    for (let i = 0; i < n; ++i) {
      if (i && i % divider === 0) {
        display += ': ';
      }
      display += data[i] + ' ';
    }
    return display;
  }

  function formatLineSensorActivation(data) {
    if (!(Array.isArray(data) && data.length)) {
      return '-';
    }
    let n = (data.length + 2) / 3;
    let divider = (n & 3) === 0 ? n >> 2 : (n & 1) === 0 ? n >> 1 : n;

    let display = '';
    for (let i = 0, j = 0; i < n; ++i) {
      if (i && i % divider === 0) {
        display += ': ';
      }
      if (i && i < n - 1) {
        display += data[j++] ? '1' : '0';
      }
      display += (data[j++] ? '1' : '0') + (data[j++] ? '1' : '0') + ' ';
    }
    return display;
  }

  // callback
  function hardwareTestCallback(data) {
    updateIo(data.io);
    updateLineSensor(data.line);
    updateMotor(data.motor);
    updateObstacleSensor(data.laser);
    updatePanel(data.panel);
    updatePower(data.power);
    updateRfid(data.rfid);
    updateSafety(data.safety);
    updateYaw(data.yaw);
  }

  onMount(() => {
    tab = 1;
    tabIo.port = 1;
    hardwareTest.subscribe(hardwareTestCallback);

    return () => {
      hardwareTest.unsubscribe(hardwareTestCallback);
    };
  });

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module<br/>"Hardware Test" ?',
      response: (r) => {
        if (r) {
          hardwareTest.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }

  const gridStretch = 'grid grid-cols-4 md:grid-cols-8 place-content-stretch';
  const statusBtn = 'min-h-14 overflow-hidden white-space-noraml leading-5';
  const cardStyle = 'card shadow-md';
  const cardHeaderStyle = 'card-header w-full pb-2 pt-2 text-center text-md tracking-widest';
  const cardBodyStyle = 'px-2 py-2 pt-2';
</script>

<AppLayout title="Hardware Test" moduleID="hardware-test" on:close={shutdown}>
  <div class="grow p-4">
    <TabGroup>
      <Tab bind:group={tab} name="tab1" value={1}>IO</Tab>
      <Tab bind:group={tab} name="tab2" value={2}>LED</Tab>
      <Tab bind:group={tab} name="tab3" value={3}>Line Sensor</Tab>
      <Tab bind:group={tab} name="tab4" value={4}>Motor</Tab>
      <Tab bind:group={tab} name="tab5" value={5}>Obstacle Sensor</Tab>
      <Tab bind:group={tab} name="tab6" value={6}>Panel</Tab>
      <Tab bind:group={tab} name="tab7" value={7}>Power</Tab>
      <Tab bind:group={tab} name="tab8" value={8}>RFID</Tab>
      <Tab bind:group={tab} name="tab9" value={9}>Safety</Tab>
      <Tab bind:group={tab} name="tab10" value={10}>Yaw</Tab>
      <!-- Tab Panels --->
      <svelte:fragment slot="panel">
        {#if tab === 1}
          <div
            class="variant-filled btn-group my-5 hidden w-full justify-between md:inline-flex">
            {#each [1, 2, 3, 4, 5, 6, 7, 8] as i}
              <button
                type="button"
                class:disabled={tabIo.port === i}
                class:variant-filled-secondary={tabIo.port === i}
                class="w-full"
                on:click={() => (tabIo.port = i)}>Port {i}</button>
            {/each}
          </div>
          <div
            class="variant-filled btn-group my-5 mb-1 inline-flex w-full justify-between md:hidden">
            {#each [1, 2, 3, 4] as i}
              <button
                type="button"
                class:disabled={tabIo.port === i}
                class:variant-filled-secondary={tabIo.port === i}
                class="w-full"
                on:click={() => (tabIo.port = i)}>Port {i}</button>
            {/each}
          </div>
          <div
            class="variant-filled btn-group my-5 mt-0 inline-flex w-full justify-between md:hidden">
            {#each [5, 6, 7, 8] as i}
              <button
                type="button"
                class:disabled={tabIo.port === i}
                class:variant-filled-secondary={tabIo.port === i}
                class="w-full"
                on:click={() => (tabIo.port = i)}>Port {i}</button>
            {/each}
          </div>
          <div class="overflow-hidden rounded-xl">
            <div class={gridStretch}>
              {#each [15, 14, 13, 12, 11, 10, 9, 8] as pin}
                <button
                  type="button"
                  class="p-2 {statusBtn} {io.getOutput(tabIo.port, pin)
                    ? 'variant-filled-success'
                    : 'variant-filled-error'}"
                  on:click={() => io.toggleOutput(tabIo.port, pin)}>
                  {'O' + pin}<br /><small>{io.getOutputName(tabIo.port, pin)}</small>
                </button>
              {/each}
            </div>
            <div class={gridStretch}>
              {#each [7, 6, 5, 4, 3, 2, 1, 0] as pin}
                <button
                  type="button"
                  class="p-2 {statusBtn} {io.getOutput(tabIo.port, pin)
                    ? 'variant-filled-success'
                    : 'variant-filled-error'}"
                  on:click={() => io.toggleOutput(tabIo.port, pin)}>
                  {'O' + pin}<br /><small>{io.getOutputName(tabIo.port, pin)}</small>
                </button>
              {/each}
            </div>
          </div>
          <div class="mb-2 mt-2 overflow-hidden rounded-xl">
            <div class={gridStretch}>
              {#each [15, 14, 13, 12, 11, 10, 9, 8] as pin}
                <button
                  type="button"
                  class="p-2 {statusBtn} {io.getInput(tabIo.port, pin)
                    ? 'variant-filled-success'
                    : 'variant-filled-error'}">
                  {'I' + pin}<br /><small>{io.getInputName(tabIo.port, pin)}</small>
                </button>
              {/each}
            </div>
            <div class={gridStretch}>
              {#each [7, 6, 5, 4, 3, 2, 1, 0] as pin}
                <button
                  type="button"
                  class="p-2 {statusBtn} {io.getInput(tabIo.port, pin)
                    ? 'variant-filled-success'
                    : 'variant-filled-error'}">
                  {'I' + pin}<br /><small>{io.getInputName(tabIo.port, pin)}</small>
                </button>
              {/each}
            </div>
          </div>
        {:else if tab === 2}
          <div class="space-y-2">
            <div>
              <button
                type="button"
                on:click={() => led.setMode(0)}
                class="btn mb-1 {led.mode === 0
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Off</button>
              <button
                type="button"
                on:click={() => led.setMode(1)}
                class="btn mb-1 {led.mode === 1
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Normal</button>
              <button
                type="button"
                on:click={() => led.setMode(2)}
                class="btn mb-1 {led.mode === 2
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Normal Left</button>
              <button
                type="button"
                on:click={() => led.setMode(3)}
                class="btn mb-1 {led.mode === 3
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Normal Right</button>
              <button
                type="button"
                on:click={() => led.setMode(4)}
                class="btn mb-1 {led.mode === 4
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Normal Warning</button>
            </div>
            <div>
              <button
                type="button"
                on:click={() => led.setMode(5)}
                class="btn mb-1 {led.mode === 5
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Station Wait</button>
              <button
                type="button"
                on:click={() => led.setMode(6)}
                class="btn mb-1 {led.mode === 6
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Error</button>
              <button
                type="button"
                on:click={() => led.setMode(7)}
                class="btn mb-1 {led.mode === 7
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Charging</button>
              <button
                type="button"
                on:click={() => led.setMode(8)}
                class="btn mb-1 {led.mode === 8
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Action</button>
              <button
                type="button"
                on:click={() => led.setMode(9)}
                class="btn mb-1 {led.mode === 9
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Paused</button>
              <button
                type="button"
                on:click={() => led.setMode(10)}
                class="btn mb-1 {led.mode === 10
                  ? 'variant-filled-success'
                  : 'variant-filled-surface'}">Safety Triggerred</button>
            </div>
          </div>
        {:else if tab === 3}
          <div class="grid grid-cols-1 gap-4 overflow-auto pb-2 lg:grid-cols-2">
            <div class={cardStyle}>
              <header class={cardHeaderStyle}>Front Line Sensor</header>
              <hr />
              <section class={cardBodyStyle}>
                <div class="grid grid-cols-1 gap-2 md:grid-cols-2">
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        <tr>
                          <td>Enabled</td>
                          <td>{lineSensor.data.front_enable}</td>
                        </tr>
                        <tr>
                          <td>Linear Error</td>
                          <td>{parseFloat(lineSensor.data.front_linear_error).toFixed(4)}</td>
                        </tr>
                        <tr>
                          <td>Angular Error</td>
                          <td>{parseFloat(lineSensor.data.front_angular_error).toFixed(4)}</td>
                        </tr>
                      </tbody>
                    </table>
                  </div>
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        <tr>
                          <td>Sensor Error</td>
                          <td>{lineSensor.data.front_error}</td>
                        </tr>
                        <tr>
                          <td>Out of Line</td>
                          <td>{lineSensor.data.front_out_of_line}</td>
                        </tr>
                        <tr>
                          <td>On Junction</td>
                          <td>{lineSensor.data.front_on_junction}</td>
                        </tr>
                      </tbody>
                    </table>
                  </div>
                </div>
                <div class="p-2">
                  Raw Data: {formatLineSensorRaw(lineSensor.data.front_raw)}
                </div>
                <div class="p-2">
                  Activation: {formatLineSensorActivation(lineSensor.data.front_activation)}
                </div>
              </section>
            </div>
            <div class={cardStyle}>
              <header class={cardHeaderStyle}>Rear Line Sensor</header>
              <hr />
              <section class={cardBodyStyle}>
                <div class="grid grid-cols-1 gap-2 md:grid-cols-2">
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        <tr>
                          <td>Enabled</td>
                          <td>{lineSensor.data.rear_enable}</td>
                        </tr>
                        <tr>
                          <td>Linear Error</td>
                          <td>{parseFloat(lineSensor.data.rear_linear_error).toFixed(4)}</td>
                        </tr>
                        <tr>
                          <td>Angular Error</td>
                          <td>{parseFloat(lineSensor.data.rear_angular_error).toFixed(4)}</td>
                        </tr>
                      </tbody>
                    </table>
                  </div>
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        <tr>
                          <td>Sensor Error</td>
                          <td>{lineSensor.data.rear_error}</td>
                        </tr>
                        <tr>
                          <td>Out of Line</td>
                          <td>{lineSensor.data.rear_out_of_line}</td>
                        </tr>
                        <tr>
                          <td>On Junction</td>
                          <td>{lineSensor.data.rear_on_junction}</td>
                        </tr>
                      </tbody>
                    </table>
                  </div>
                </div>
                <div class="p-2">
                  Raw Data: {formatLineSensorRaw(lineSensor.data.rear_raw)}
                </div>
                <div class="p-2">
                  Activation: {formatLineSensorActivation(lineSensor.data.rear_activation)}
                </div>
              </section>
            </div>
          </div>
        {:else if tab === 4}
          <div class="grid w-full grid-cols-1 gap-4 pb-2 lg:grid-cols-[1fr_auto]">
            <div class="grid min-w-[300px] grow grid-cols-3 gap-2">
              <div class="flex flex-col place-content-around space-y-4">
                <div class="flex justify-center">
                  <button
                    type="button"
                    class="variant-filled-warning btn"
                    on:touchstart={() => motor.rightReverse()}
                    on:touchend={() => motor.rightRelease()}
                    on:mousedown={() => motor.rightReverse()}
                    on:mouseup={() => motor.rightRelease()}>
                    <i class="fa-solid fa-chevron-up mr-2"></i> Right
                  </button>
                </div>
                <div class="flex justify-center">
                  <button
                    type="button"
                    class="variant-filled-warning btn"
                    on:touchstart={() => motor.rightForward()}
                    on:touchend={() => motor.rightRelease()}
                    on:mousedown={() => motor.rightForward()}
                    on:mouseup={() => motor.rightRelease()}>
                    <i class="fa-solid fa-chevron-down mr-2"></i> Right
                  </button>
                </div>
                <div class="text-center">{parseFloat(motor.rightSpeed).toFixed(2)} m/s</div>
              </div>
              <div class="flex flex-col justify-center pb-20">
                <button
                  type="button"
                  class="variant-filled-error btn"
                  style="min-height: 80px;"
                  on:touchstart={() => motor.stop()}
                  on:mousedown={() => motor.stop()}>
                  Stop
                </button>
              </div>
              <div class="flex flex-col place-content-around space-y-4">
                <div class="flex justify-center">
                  <button
                    type="button"
                    class="variant-filled-warning btn"
                    on:touchstart={() => motor.leftReverse()}
                    on:touchend={() => motor.leftRelease()}
                    on:mousedown={() => motor.leftReverse()}
                    on:mouseup={() => motor.leftRelease()}>
                    Left <i class="fa-solid fa-chevron-up ml-2"></i>
                  </button>
                </div>
                <div class="flex justify-center">
                  <button
                    type="button"
                    class="variant-filled-warning btn"
                    on:touchstart={() => motor.leftForward()}
                    on:touchend={() => motor.leftRelease()}
                    on:mousedown={() => motor.leftForward()}
                    on:mouseup={() => motor.leftRelease()}>
                    Left <i class="fa-solid fa-chevron-down ml-2"></i>
                  </button>
                </div>
                <div class="text-center">{parseFloat(motor.leftSpeed).toFixed(2)} m/s</div>
              </div>
            </div>
            <div class={cardStyle}>
              <header class={cardHeaderStyle}>Motor Status</header>
              <hr />
              <section class={cardBodyStyle}>
                <div class="grid grid-cols-1 gap-2 md:grid-cols-2">
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        <tr>
                          <td>Left Distance (m)</td>
                          <td>{parseFloat(motor.data.left_distance).toFixed(3)}</td>
                        </tr>
                        <tr>
                          <td>Left Current (A)</td>
                          <td>{parseFloat(motor.data.left_current).toFixed(3)}</td>
                        </tr>
                      </tbody>
                    </table>
                  </div>
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        <tr>
                          <td>Right Distance (m)</td>
                          <td>{parseFloat(motor.data.right_distance).toFixed(3)}</td>
                        </tr>
                        <tr>
                          <td>Right Current (A)</td>
                          <td>{parseFloat(motor.data.right_current).toFixed(3)}</td>
                        </tr>
                      </tbody>
                    </table>
                  </div>
                </div>
                <div class="p-2 pt-4">
                  Straight Distance (m): {parseFloat(motor.data.straight_distance).toFixed(3)}
                </div>
                <div class="p-2">
                  Rotational Distance (rad): {parseFloat(
                    motor.data.rotational_distance
                  ).toFixed(3)}
                </div>
              </section>
            </div>
          </div>
        {:else if tab === 5}
          <div class="grid w-full grid-cols-1 gap-4 overflow-auto pb-2 lg:grid-cols-[3fr_2fr]">
            <div class="flex flex-col place-content-around">
              <div class="pb-4">
                <div class="flex items-center justify-center pb-2">
                  Profile: {selectedObstacleSensorProfile}
                </div>
                <div class="flex flex-wrap gap-4">
                  {#each [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] as i}
                    <button
                      type="button"
                      on:click={() => {
                        selectedObstacleSensorProfile = i;
                        obstacleSensor.setProfile(selectedObstacleSensorProfile);
                      }}
                      class="min-w-[60px] max-w-[82px] flex-grow {i ===
                      selectedObstacleSensorProfile
                        ? 'variant-filled-warning'
                        : 'variant-filled-surface'} btn rounded-md">
                      {i}
                    </button>
                  {/each}
                </div>
              </div>
              <hr />
              {#if obstacleSensor.profile === 0}
                <div class="py-4">
                  <div class="flex items-center justify-center pb-2">
                    Area: {selectedObstacleSensorArea}
                  </div>
                  <div class="flex flex-wrap gap-4 py-4">
                    {#each [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31] as i}
                      <button
                        type="button"
                        on:click={() => {
                          selectedObstacleSensorArea = i;
                          obstacleSensor.setArea(selectedObstacleSensorArea);
                        }}
                        class="{i === selectedObstacleSensorArea
                          ? 'variant-filled-warning'
                          : 'variant-filled-surface'} btn rounded-md">
                        {i}
                      </button>
                    {/each}
                  </div>
                </div>
              {:else}
                <div class="py-4">
                  <div class="flex items-center justify-center pb-2">
                    Area: {selectedObstacleSensorArea}
                  </div>
                  <div class="flex flex-wrap gap-4 py-4">
                    {#each obstacleOptions as [i, n]}
                      <button
                        type="button"
                        on:click={() => {
                          selectedObstacleSensorArea = i;
                          obstacleSensor.setArea(selectedObstacleSensorArea);
                        }}
                        class="{i === selectedObstacleSensorArea
                          ? 'variant-filled-warning'
                          : 'variant-filled-surface'} btn rounded-md">
                        {n}
                      </button>
                    {/each}
                  </div>
                </div>
              {/if}
            </div>
            <div class={cardStyle}>
              <header class={cardHeaderStyle}>Obstacle Sensor</header>
              <hr />
              <section class={cardBodyStyle}>
                <div class="table-container">
                  <table class="table">
                    <tbody>
                      <tr>
                        <td>Far</td>
                        <td>{obstacleSensor.data.far_blocked}</td>
                      </tr>
                      <tr>
                        <td>Middle</td>
                        <td>{obstacleSensor.data.middle_blocked}</td>
                      </tr>
                      <tr>
                        <td>Near</td>
                        <td>{obstacleSensor.data.near_blocked}</td>
                      </tr>
                      <tr>
                        <td>Malfunction</td>
                        <td>{obstacleSensor.data.malfunction}</td>
                      </tr>
                      <tr class="">
                        <td>Hint</td>
                        <td>{obstacleSensor.data.hint}</td>
                      </tr>
                    </tbody>
                  </table>
                </div>
              </section>
            </div>
          </div>
        {:else if tab === 6}
          <div class="grid w-full grid-cols-1 gap-4 overflow-auto pb-2 md:grid-cols-2">
            <div class="space-y-1">
              <div class="flex justify-center font-bold">Panel LEDs</div>
              <button
                type="button"
                class="btn w-full {panel.leds['start']
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                on:click={() => panel.toggleLed('start')}>Start</button>
              <button
                type="button"
                class="btn w-full {panel.leds['stop']
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                on:click={() => panel.toggleLed('stop')}>Stop</button>
              <button
                type="button"
                class="btn w-full {panel.leds['mode']
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                on:click={() => panel.toggleLed('mode')}>Mode</button>
              <button
                type="button"
                class="btn w-full {panel.leds['unbrake']
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                on:click={() => panel.toggleLed('unbrake')}>Unbrake</button>
              <button
                type="button"
                class="btn w-full {panel.leds['power']
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                on:click={() => panel.toggleLed('power')}>Power</button>
              <button
                type="button"
                class="btn w-full {panel.leds['low_batt']
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                on:click={() => panel.toggleLed('low_batt')}>Low Batt</button>
            </div>
            <div class={cardStyle}>
              <header class={cardHeaderStyle}>Panel Status</header>
              <hr />
              <section class={cardBodyStyle}>
                <div class="grid grid-cols-1 gap-2">
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        <tr>
                          <td>Start Button</td>
                          <td>{panel.data.button_start}</td>
                        </tr>
                        <tr>
                          <td>Stop Button</td>
                          <td>{panel.data.button_stop}</td>
                        </tr>
                        <tr>
                          <td>Mode Button</td>
                          <td>{panel.data.button_mode}</td>
                        </tr>
                        <tr>
                          <td>Unbrake Button</td>
                          <td>{panel.data.button_unbrake}</td>
                        </tr>
                        <tr>
                          <td>Power Button</td>
                          <td>{panel.data.button_power}</td>
                        </tr>
                      </tbody>
                    </table>
                  </div>
                </div>
              </section>
            </div>
          </div>
        {:else if tab === 7}
          <div class={cardStyle}>
            <header class={cardHeaderStyle}>Power</header>
            <hr />
            <section class={cardBodyStyle}>
              <div class="grid grid-cols-1 gap-2 md:grid-cols-2">
                <div class="table-container">
                  <table class="table">
                    <tbody>
                      <tr>
                        <td>Charging</td>
                        <td>{power.data.charging}</td>
                      </tr>
                      <tr>
                        <td>Battery 1 Voltage (V)</td>
                        <td>{parseFloat(power.data.battery1_voltage).toFixed(2)}</td>
                      </tr>
                      <tr>
                        <td>Battery 2 Voltage (V)</td>
                        <td>{parseFloat(power.data.battery2_voltage).toFixed(2)}</td>
                      </tr>
                      <tr>
                        <td>Battery Current (A)</td>
                        <td>{parseFloat(power.data.battery_current).toFixed(3)}</td>
                      </tr>
                    </tbody>
                  </table>
                </div>
                <div class="table-container">
                  <table class="table">
                    <tbody>
                      <tr>
                        <td>Auto Charger Voltage (V)</td>
                        <td>{parseFloat(power.data.auto_charger_voltage).toFixed(2)}</td>
                      </tr>
                      <tr>
                        <td>Manual Charger Voltage (V)</td>
                        <td>{parseFloat(power.data.manual_charger_voltage).toFixed(2)}</td>
                      </tr>
                      <tr>
                        <td>Manual Charger Connected</td>
                        <td>{power.data.manual_charger_connected}</td>
                      </tr>
                    </tbody>
                  </table>
                </div>
              </div>
            </section>
          </div>
        {:else if tab === 8}
          <div class={cardStyle}>
            <header class={cardHeaderStyle}>RFID</header>
            <hr />
            <section class={cardBodyStyle}>
              <div class="table-container">
                <table class="table">
                  <tbody>
                    <tr>
                      <td>Front RFID</td>
                      <td>{rfid.data.front_rfid || '-'}</td>
                    </tr>
                    <tr>
                      <td>Rear FRID</td>
                      <td>{rfid.data.rear_rfid || '-'}</td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </section>
          </div>
        {:else if tab === 9}
          <div class={cardStyle}>
            <header class={cardHeaderStyle}>Safety</header>
            <hr />
            <section class={cardBodyStyle}>
              <div class="grid grid-cols-1 gap-2 md:grid-cols-2">
                <div class="table-container">
                  <table class="table">
                    <tbody>
                      <tr>
                        <td>Bumper Front</td>
                        <td>{safety.data.bumper_front}</td>
                      </tr>
                      <tr>
                        <td>Bumper Rear</td>
                        <td>{safety.data.bumper_rear}</td>
                      </tr>
                      <tr>
                        <td>Emergency Button</td>
                        <td>{safety.data.emergency_button}</td>
                      </tr>
                      <tr>
                        <td>Safety External 1</td>
                        <td>{safety.data.safety_external1}</td>
                      </tr>
                      <tr>
                        <td>Safety External 2</td>
                        <td>{safety.data.safety_external2}</td>
                      </tr>
                    </tbody>
                  </table>
                </div>
                <div class="table-container">
                  <table class="table">
                    <tbody>
                      <tr>
                        <td>Motor Fault</td>
                        <td>{safety.data.motor_fault}</td>
                      </tr>
                      <tr>
                        <td>Wheel Slippage</td>
                        <td>{safety.data.wheel_slippage}</td>
                      </tr>
                      <tr>
                        <td>Charger Connected</td>
                        <td>{safety.data.charger_connected}</td>
                      </tr>
                      <tr>
                        <td>Navigation Trigger</td>
                        <td>{safety.data.nav_trigger}</td>
                      </tr>
                      <tr>
                        <td>Time Jump</td>
                        <td>{safety.data.time_jump}</td>
                      </tr>
                    </tbody>
                  </table>
                </div>
              </div>
            </section>
          </div>
        {:else if tab === 10}
          <div class="grid grid-cols-1 gap-4 overflow-auto pb-2 md:grid-cols-2">
            <div class="flex w-full place-content-around items-center">
              <button
                type="button"
                class="variant-filled-warning btn p-8"
                on:touchstart={() => yaw.left()}
                on:touchend={() => yaw.release()}
                on:mousedown={() => yaw.left()}
                on:mouseup={() => yaw.release()}>
                <i class="fa-solid fa-rotate-left"></i>
              </button>
              <button
                type="button"
                class="variant-filled-error btn p-8"
                on:touchstart={() => yaw.stop()}
                on:mousedown={() => yaw.stop()}>
                <i class="fa-solid fa-hand"></i>
              </button>
              <button
                type="button"
                class="variant-filled-warning btn p-8"
                on:touchstart={() => yaw.right()}
                on:touchend={() => yaw.release()}
                on:mousedown={() => yaw.right()}
                on:mouseup={() => yaw.release()}>
                <i class="fa-solid fa-rotate-right"></i>
              </button>
            </div>
            {#if yaw.data}
              <div class={cardStyle}>
                <header class={cardHeaderStyle}>Yaw</header>
                <hr />
                <section class={cardBodyStyle}>
                  <div class="table-container">
                    <table class="table">
                      <tbody>
                        {#each yaw.data as pair}
                          <tr>
                            <td>{pair[0]}</td>
                            <td>{parseFloat(pair[1]).toFixed(1)}°</td>
                          </tr>
                        {/each}
                      </tbody>
                    </table>
                  </div>
                </section>
              </div>
            {/if}
          </div>
        {/if}
      </svelte:fragment>
    </TabGroup>
  </div>
</AppLayout>

<style>
  button {
    outline: none !important;
  }
</style>
