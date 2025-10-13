<svelte:options accessors />

<script>
  import { createEventDispatcher, onMount } from 'svelte';
  import { popup, focusTrap, ListBox, ListBoxItem } from '@skeletonlabs/skeleton';
  import { get } from 'svelte/store';
  import { selectTargetCount } from 'stores/utils.js';
  import { twMerge } from 'tailwind-merge';
  import * as _ from 'lodash-es';

  const dispatch = createEventDispatcher();

  let className = '';
  export { className as class };
  export let buttonClass = '';
  export let buttonTitle = '';
  export let name = '';
  export let options = [];
  export let value = '';
  export let startEmpty = false;
  export let enableFilter = true;
  export let multiple = false;
  export let disabled = false;
  export let placement = 'bottom';
  export let offset = 10;

  export function getDisplayVal() {
    return displayVal;
  }

  export function checkValExist(val) {
    for (const opts of options) {
      for (const [v, _] of opts.options) {
        if (v === val) {
          return true;
        }
      }
    }
  }

  export function trigger(event) {
    dispatch(event);
  }

  export function show() {
    const e = new Event('focus');
    btnDom.dispatchEvent(e);
  }

  if (multiple && !Array.isArray(value)) {
    value = value ? [value] : [];
  }

  let filterTxt = '';
  let btnDom;
  let popupDom;
  let filterDom;
  let displayVal = '-----------';
  let focus = false;
  let _options = [];
  let filterHeight = 82;
  let requiredError = ''; // double as indicator and css class

  selectTargetCount.update((c) => c + 1);

  // generate unique target since cannot pass dom ref.
  const targetClass = `select-${get(selectTargetCount)}`;

  const popupSettings = {
    event: 'focus-click',
    target: targetClass,
    placement: placement,
    closeQuery: multiple ? '' : '.listbox-item',
    state: (e) => onPopupState(e.state),
    middleware: {
      size: {
        padding: 30,
        apply({ availableWidth, availableHeight }) {
          if (!popupDom) {
            return;
          }
          Object.assign(popupDom.style, {
            maxWidth: `${availableWidth}px`,
            maxHeight: `${availableHeight - filterHeight}px`
          });
        }
      },
      offset
    }
  };

  function updateOptions(opts, filter) {
    if (!Array.isArray(opts)) {
      return;
    }
    if (opts.length > 0 && Array.isArray(opts[0])) {
      opts = [
        {
          name: '_',
          options: opts // [[val, disp, isDisabled]]
        }
      ];
    }
    if (filter) {
      let filterW = filter.trim().toLowerCase();
      let filteredOpts = [];
      for (const o of opts) {
        let optsF = {
          name: o.name,
          options: []
        };
        for (const v of o.options) {
          let searchable = `${opts.name !== '_' ? opts.name : ''} ${v[1]}`;
          searchable = searchable.trim().toLowerCase();
          if (searchString(filterW, searchable)) {
            optsF.options.push(v);
          }
        }
        filteredOpts.push(optsF);
      }
      _options = filteredOpts;
    } else {
      _options = opts;
    }
    if (!value && !startEmpty) {
      for (const o of _options) {
        for (const [v, _n] of o.options) {
          if (v) {
            value = v;
            return;
          }
        }
      }
    }
  }

  $: updateOptions(options, filterTxt);

  function updateDisplayVal(val) {
    requiredError = '';
    displayVal = '-----------';
    if (multiple && val.length > 1) {
      displayVal = `${val.length} items selected`;
      return;
    }
    if (Array.isArray(val)) {
      val = val[0];
    }
    for (const opts of _options) {
      for (const [v, t] of opts.options) {
        if (v === val) {
          displayVal = t;
          return;
        }
      }
    }
  }

  $: updateDisplayVal(value);

  function searchString(words, searchable) {
    // TODO: use string similarity algo?
    if (searchable.indexOf(words) >= 0) {
      return true;
    }
  }

  function onPopupState(state) {
    focus = state;
    if (!state) {
      filterTxt = '';
    }
    dispatch(state ? 'open' : 'close');
  }

  function onClick() {
    // TODO: this hack to force set focus manually. check when popup set focus.
    if (filterDom) {
      setTimeout(() => {
        filterDom.focus();
      }, 100);
    }
  }

  function selectAll() {
    const filteredValues = _.flatMap(_options, (os) => os.options.map((o) => o[0]));
    const combinedArray = [...value, ...filteredValues];
    const uniqueValues = new Set(combinedArray);
    value = [...uniqueValues];
    dispatch('change');
  }

  function deselectAll() {
    const filteredValues = _.flatMap(_options, (os) => os.options.map((o) => o[0]));
    value = value.filter((v) => !filteredValues.includes(v));
    dispatch('change');
  }

  // TODO: fix select for form usecase.
  if ('required' in $$props) {
    onMount(() => {
      const form = btnDom.closest('form');
      if (form) {
        upgradeFormValidation(form);
        form.addValidity(onValidation);
        form.addReportValidity(onReportValidity);
      }
      return () => {
        if (form) {
          form.removeValidity(onValidation);
          form.removeReportValidity(onReportValidity);
        }
      };
    });
  }

  function onValidation() {
    // only called if required.
    return isValid();
  }

  function onReportValidity() {
    if (isValid()) {
      return true;
    }
    // TODO: report validity here.
    btnDom.scrollIntoView();

    return false;
  }

  function isValid() {
    let v = false;
    if (Array.isArray(value)) {
      v = value.length > 0;
    } else {
      v = !!value;
    }

    requiredError = v ? '' : '!bg-red-100 !ring-red-500';

    return v;
  }

  function upgradeFormValidation(form) {
    // HACK: to allow for custom validation
    if (form.upgradeFormValidation) {
      return;
    }
    form.upgradeFormValidation = true;
    form.customValidity = new Set();
    form.customReportValidity = new Set();

    const originalCheckValidity = form.checkValidity;
    const originalReportValidity = form.reportValidity;
    form.checkValidity = function () {
      if (!originalCheckValidity.call(this)) {
        return false;
      }

      for (const v of this.customValidity) {
        if (!v()) {
          return false;
        }
      }
      return true;
    };
    form.addValidity = function (v) {
      this.customValidity.add(v);
    };
    form.removeValidity = function (v) {
      this.customValidity.delete(v);
    };
    form.reportValidity = function () {
      if (!originalReportValidity.call(this)) {
        return false;
      }

      for (const v of this.customReportValidity) {
        if (!v()) {
          return false;
        }
      }
      return true;
    };
    form.addReportValidity = function (v) {
      this.customReportValidity.add(v);
    };
    form.removeReportValidity = function (v) {
      this.customReportValidity.delete(v);
    };
  }
</script>

<button
  type="button"
  class={twMerge(
    'variant-filled btn w-[400px] justify-between',
    className,
    buttonClass,
    requiredError
  )}
  use:popup={popupSettings}
  on:click={onClick}
  bind:this={btnDom}
  title={buttonTitle}
  data-for-popup={targetClass}
  {disabled}>
  <span
    class="w-full overflow-hidden overflow-ellipsis whitespace-nowrap text-left capitalize">
    {displayVal ?? '-----------'}
  </span>
  <div class="relative float-right">
    <i class="fa-solid fa-caret-down"></i>
  </div>
</button>
<div
  class={twMerge('card fixed z-30 w-[400px] py-2 shadow-xl', className)}
  data-popup={targetClass}
  use:focusTrap={focus}>
  {#if enableFilter}
    <header class="card-header border-b-2 py-3">
      <input class="input rounded" type="text" bind:value={filterTxt} bind:this={filterDom} />
    </header>
  {/if}
  {#if multiple}
    <div class="my-2 grid grid-cols-2">
      <button type="button" class="variant-form btn m-1 rounded-md p-1" on:click={selectAll}>
        Select All
      </button>
      <button type="button" class="variant-form btn m-1 rounded-md p-1" on:click={deselectAll}>
        Deselect All
      </button>
    </div>
  {/if}
  <div class="overflow-y-scroll" bind:this={popupDom}>
    <ListBox rounded="rounded-none" {multiple} {disabled}>
      {#each _options as o, i}
        {#if i > 0}
          <hr class="pt-3" />
        {/if}
        {#if o.name !== '_'}
          <span class="px-3 py-5 text-sm font-bold tracking-widest opacity-30">{o.name}</span>
        {/if}
        {#each o.options as opt}
          {#if opt[2]}
            <span class="block w-full cursor-not-allowed select-none px-4 py-2 opacity-40">
              {#if opt[1] === ''}
                <span class="opacity-40">(empty)</span>
              {:else}
                {opt[1]}
              {/if}
            </span>
          {:else}
            <ListBoxItem bind:group={value} {name} value={opt[0]} on:change>
              {#if opt[1] === ''}
                <span class="opacity-40">(empty)</span>
              {:else}
                {opt[1]}
              {/if}
            </ListBoxItem>
          {/if}
        {/each}
      {/each}
    </ListBox>
  </div>
  <div class="bg-surface-100-800-token arrow" />
</div>
