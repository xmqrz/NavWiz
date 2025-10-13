<script>
  import { whiteLabel } from 'stores/white-label.js';
  import { title as pgTitle } from 'stores/page-title';
  import ValidationMessage from 'components/ValidationMessage.svelte';

  export let title = '(Title Missing)';
  export let back = [];
  export let validation = true;
  pgTitle.set(title);
</script>

<slot name="header">
  <div class="p-5">
    <div class="flex space-x-5 py-5">
      {#if back.length}
        <a href={back[1]} class="variant-filled-secondary btn">
          <i class="fa-solid fa-chevron-left mr-2"></i>
          {back[0]}
        </a>
      {/if}
      <h1 class="text-3xl tracking-widest">{title}</h1>
    </div>
    <hr />
  </div>
</slot>
<div class="p-5">
  {#if validation}
    <ValidationMessage />
  {/if}
  <slot />
</div>
<slot name="footer">
  <div class="p-5 pt-20">
    <hr />
    <!-- eslint-disable-next-line svelte/no-at-html-tags -->
    <p class="pt-5">NavWiz ConfigPanel. {@html $whiteLabel.copyright_label || ''}</p>
  </div>
</slot>
