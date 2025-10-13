<script>
  import { twMerge } from 'tailwind-merge';

  export let name;
  export let id = undefined;
  export let accept = '';
  export let multiple = false;
  export let disabled = false;
  export let value = undefined; // list of url
  export let filename = undefined;
  let className = '';
  export { className as class };

  let _value;
  let clear;
  let inputVal;
  let input;

  function initValue(v) {
    clear = false;
    clearInput();
    if (!v) {
      _value = [];
    } else if (!Array.isArray(v)) {
      _value = [v];
    } else {
      _value = v;
    }
  }

  $: initValue(value);

  function onClear() {
    if (clear) {
      clearInput();
    }
  }

  function onChange() {
    if (inputVal) {
      clear = false;
    }
  }

  export function clearInput() {
    // TODO: impove so that no need to manual call.

    // TODO: why need to set both?
    // set clear the actual input
    if (input) {
      input.value = '';
    }

    // clear value tracking for display
    inputVal = undefined;
  }

  function urlFilename(url, i) {
    let n;
    if (filename) {
      if (Array.isArray(filename)) {
        n = filename[i];
      } else {
        n = filename;
      }
    }
    return n || decodeURIComponent(url.split('/').pop());
  }
</script>

{#if _value && _value.length > 0}
  <div class="mb-1 w-full space-x-3">
    {#each _value as url, i}
      <a class="anchor {inputVal || clear ? 'line-through' : ''}" href={url}>
        {urlFilename(url, i)}
      </a>
    {/each}
    <label class="label inline-block cursor-pointer select-none">
      <input
        name={clear ? name : ''}
        class="checkbox"
        type="checkbox"
        bind:checked={clear}
        on:change|preventDefault={onClear}
        {disabled} />
      <span class="text-lg font-semibold">clear</span>
    </label>
  </div>
{/if}
<div class="w-full">
  <input
    {id}
    name={inputVal ? name : ''}
    {accept}
    {multiple}
    class={twMerge('input px-7', className)}
    type="file"
    bind:this={input}
    bind:value={inputVal}
    on:change|preventDefault={onChange}
    {disabled} />
</div>
