<script>
  import { keyboardState } from './controller.js';
</script>

<div
  class:bottom-[360px]={$keyboardState}
  class="scrollbar-hide fixed inset-0 grid place-items-center overflow-auto py-3 transition-[bottom] delay-150 duration-150">
  <slot />
</div>
