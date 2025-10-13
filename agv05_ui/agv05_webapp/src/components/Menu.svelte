<svelte:options accessors />

<script>
  import { popup } from '@skeletonlabs/skeleton';
  import { get } from 'svelte/store';
  import { selectTargetCount } from 'stores/utils.js';
  import { twMerge } from 'tailwind-merge';

  export let btnClass = '';
  export let disabled = false;
  export let placement = 'bottom';
  export let text = 'Menu';
  export let closeQuery = '.menu-close';
  export let event = 'focus-click';

  selectTargetCount.update((c) => c + 1);

  // generate unique target since cannot pass dom ref.
  const targetClass = `select-${get(selectTargetCount)}`;

  const popupSettings = {
    target: targetClass,
    event,
    placement,
    closeQuery
  };
</script>

<button
  type="button"
  class={twMerge('btn [&>*]:pointer-events-none', btnClass)}
  use:popup={popupSettings}
  data-for-popup={targetClass}
  on:click|preventDefault
  {disabled}>
  <slot name="text">
    <span>{text}</span>
  </slot>
</button>
<div data-popup={targetClass}>
  <div class="card z-30 py-2 shadow-xl">
    <slot />
  </div>
</div>
