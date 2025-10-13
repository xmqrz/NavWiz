<script>
  import {
    computePosition,
    autoUpdate,
    offset,
    shift,
    flip,
    arrow,
    size
  } from '@floating-ui/dom';
  import {
    Drawer,
    Modal,
    Toast,
    getDrawerStore,
    getModalStore,
    initializeStores,
    storePopup
  } from '@skeletonlabs/skeleton';
  import { setContext, onMount } from 'svelte';

  import { user } from 'stores/auth';
  import { initTooltip, initHttp, extendModalStore } from '$lib/utils';
  import { initKeyboard } from 'components/keyboard/controller.js';

  import Login from './components/Login.svelte';
  import ModalAlternateStartMenu from './(user-panel)/components/modals/ModalAlternateStartMenu.svelte';
  import ModalLoadingSpinner from './(user-panel)/components/modals/ModalLoadingSpinner.svelte';
  import ModalPopup from './(user-panel)/components/modals/ModalPopup.svelte';
  import ModalDowntimeTracker from './(user-panel)/components/modals/ModalDowntimeTracker.svelte';
  import ModalShowHttpError from './(user-panel)/components/modals/ModalShowHttpError.svelte';
  import ModalTaskParams from './(user-panel)/components/modals/ModalTaskParams.svelte';
  import ModalResumeTransaction from './(user-panel)/components/modals/ModalResumeTransaction.svelte';
  import { MainController } from './(user-panel)/core/main.controller.js';
  import TaskRunner from './(user-panel)/(controller)/dashboard/drawer/TaskRunner/TaskRunner.svelte';
  import TaskHistory from './(user-panel)/(controller)/dashboard/drawer/TaskHistory.svelte';
  import DrawerLayout from './(user-panel)/(controller)/dashboard/drawer/components/DrawerLayout.svelte';

  import '../app.css';

  initializeStores();
  storePopup.set({ computePosition, autoUpdate, offset, shift, flip, arrow, size });

  setContext('user', user);

  // register user-panel context here for modal access.
  let mainController = new MainController();
  setContext('mainController', mainController);

  const drawerStore = getDrawerStore();
  const modalStore = getModalStore();
  extendModalStore(modalStore);

  initHttp(modalStore);

  const modalRegistry = {
    modalAlternateStartMenu: { ref: ModalAlternateStartMenu },
    modalLoadingSpinner: { ref: ModalLoadingSpinner },
    modalPopup: { ref: ModalPopup },
    modalDowntimeTracker: { ref: ModalDowntimeTracker },
    modalShowHttpError: { ref: ModalShowHttpError },
    modalTaskParams: { ref: ModalTaskParams },
    modalResumeTransaction: { ref: ModalResumeTransaction }
  };

  function handleLogin() {
    drawerStore.close();
  }

  onMount(() => {
    initTooltip();
    initKeyboard();
  });
</script>

<Modal position="items-start" components={modalRegistry} />
<Toast zIndex="z-[1001]" />
<Drawer>
  {#if $drawerStore.id === 'login'}
    <Login on:login={handleLogin} />
  {:else if $drawerStore.id === 'task-runner'}
    <DrawerLayout>
      <TaskRunner />
    </DrawerLayout>
  {:else if $drawerStore.id === 'task-history'}
    <DrawerLayout>
      <TaskHistory />
    </DrawerLayout>
  {/if}
</Drawer>
<slot />
