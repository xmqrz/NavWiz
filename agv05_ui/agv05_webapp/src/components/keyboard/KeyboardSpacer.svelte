<script>
  import fadeScale from '$lib/transitions/fade-scale.js';
  import { twMerge } from 'tailwind-merge';

  import { keyboardState } from './controller.js';

  let className = '';
  export { className as class };
</script>

{#if $keyboardState}
  {#key keyboardState}
    <div
      transition:fadeScale={{ duration: 200, noFade: true }}
      class={twMerge('h-[360px]', className)}>
    </div>
  {/key}
{/if}
