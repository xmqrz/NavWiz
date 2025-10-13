<script>
  // Hack to allow toggling between types without losing focus.
  export let type = 'password';
  export let value = '';
  let className = '';
  export { className as class };
</script>

<input class={className} bind:value {...{ type }} />
