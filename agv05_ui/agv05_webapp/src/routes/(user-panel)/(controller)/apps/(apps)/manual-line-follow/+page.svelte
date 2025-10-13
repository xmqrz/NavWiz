<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount, getContext } from 'svelte';
  import { RangeSlider } from '@skeletonlabs/skeleton';
  import { SlideToggle } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';
  import * as _ from 'lodash-es';

  import AppLayout from 'components/AppLayout.svelte';

  const mainController = getContext('mainController');
  const manualLineFollow = mainController.moduleService.getSub('manual-line-follow');
  const modalStore = getModalStore();

  let speed = {
    data: 0.3,
    previous: 0.3
  };
  let motor = {
    free: false,
    previous: false
  };
  let screenRotate = true;

  function speedUpdated() {
    if (speed.data > 1.5) {
      speed.data = 1.5;
    } else if (speed.data < 0.1) {
      speed.data = 0.1;
    }
    modalStore.trigger({
      type: 'confirm',
      title: `Confirm speed change to ${speed.data}?`,
      response: (r) => {
        if (r) {
          speed.previous = speed.data;
        } else {
          speed.data = speed.previous;
        }
      }
    });
  }

  function freeMotorUpdated() {
    modalStore.trigger({
      type: 'confirm',
      title: motor.free ? 'Confirm set motor to free?' : 'Confirm set motor to locked?',
      response: (r) => {
        if (r) {
          manualLineFollow.publish({
            command: 'free_motor',
            free_motor: motor.free
          });
          motor.previous = motor.free;
        } else {
          motor.free = motor.previous;
        }
      }
    });
  }

  let _bindings = [
    '',
    'forward',
    '',
    'rotate_left',
    'stop',
    'rotate_right',
    'uturn_left',
    'reverse',
    'uturn_right'
  ];
  let _icons = [
    '',
    'fa-arrow-up',
    '',
    'fa-arrow-left',
    'fa-hand',
    'fa-arrow-right',
    'fa-arrow-turn-up fa-rotate-180',
    'fa-arrow-down',
    'fa-arrow-turn-up fa-flip-vertical'
  ];

  function cellP(_cell) {
    /* cell pressed */
  }

  let cellR = _.debounce(function (cell) {
    /* cell released */
    if (!Number.isInteger(cell) || cell < 0 || cell >= 9) {
      return;
    }
    let nav = _bindings[cell];
    if (!nav) {
      return;
    }

    manualLineFollow.publish({
      command: 'nav',
      nav: nav,
      speed: Number.parseFloat(speed.data)
    });
  }, 100);

  let allowedMotions = [];

  function manualLineFollowCallback(data) {
    if (data.command === 'list_allowed_motions') {
      allowedMotions = data.allowed_motions;
    } else if (data.command === 'is_motor_free') {
      motor.free = data.is_motor_free;
    }
  }

  onMount(() => {
    manualLineFollow.subscribe(manualLineFollowCallback);

    // allow time for subscribe to happen first.
    setTimeout(function () {
      manualLineFollow.publish({
        command: 'list_allowed_motions'
      });
      manualLineFollow.publish({
        command: 'is_motor_free'
      });
    }, 100);

    return () => {
      manualLineFollow.unsubscribe(manualLineFollowCallback);
    };
  });

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module<br/>"Manual Line Follow" ?',
      response: (r) => {
        if (r) {
          manualLineFollow.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }
</script>

<AppLayout title="Manual Line Follow" moduleID="manual-line-follow" on:close={shutdown}>
  <div class="flex h-full flex-col gap-y-4 overflow-auto p-4">
    <div class="grid grid-cols-1 gap-4 md:grid-cols-4">
      <div class="card p-4 md:col-span-3">
        <RangeSlider
          name="speed.data"
          bind:value={speed.data}
          on:change={speedUpdated}
          min={0.1}
          max={1.5}
          step={0.1}
          ticked>
          <div class="flex items-center justify-between">
            <div class="font-bold">Speed</div>
            <div class="text-md">{parseFloat(speed.data).toFixed(1)} m/s</div>
          </div>
        </RangeSlider>
      </div>
      <div class="card flex items-center justify-center p-4 align-middle">
        <SlideToggle
          name="free-motor"
          size="sm"
          bind:checked={motor.free}
          on:change={freeMotorUpdated}>Free Motor</SlideToggle>
      </div>
    </div>
    <div class="grid flex-auto grid-cols-3 md:grid-cols-5">
      <div
        class="screen-rotate col-span-3 grid h-full max-h-64 w-full grid-cols-3 place-items-stretch gap-4 place-self-center md:col-start-2"
        class:screen-rotate-active={screenRotate}>
        {#each { length: 9 } as _, i}
          {#if _bindings[i] && (i === 4 || allowedMotions.indexOf(_bindings[i]) >= 0)}
            <button
              type="button"
              class="variant-filled-warning rounded-xl"
              class:variant-filled-error={i === 4}
              on:mousedown|preventDefault={() => cellP(i)}
              on:mouseup={() => cellR(i)}
              on:touchstart|preventDefault={() => cellP(i)}
              on:touchend={() => cellR(i)}>
              <i class="fa-solid {_icons[i]}"></i>
            </button>
          {:else}
            <div></div>
          {/if}
        {/each}
      </div>
      <div class="relative col-start-1">
        <button
          type="button"
          tip-title="Rotate manual control"
          class="variant-filled btn absolute bottom-0 left-0"
          on:click={() => (screenRotate = !screenRotate)}>
          <i class="icon fa fa-undo"></i>
        </button>
      </div>
    </div>
  </div>
</AppLayout>
