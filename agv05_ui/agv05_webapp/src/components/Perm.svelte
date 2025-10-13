<script>
  import { derived } from 'svelte/store';
  import { getContext } from 'svelte';

  // NOTE: cannot use both at the same time
  export let perms = [];
  export let somePerms = []; // allow if any one of the param exist

  if (!Array.isArray(perms)) {
    perms = [perms];
  }

  if (!Array.isArray(somePerms)) {
    somePerms = [somePerms];
  }

  const user = getContext('user');
  const allowed = derived(user, ($user) => {
    if (perms.length > 0) {
      return Boolean(
        $user.is_superuser ||
          ($user.permissions && perms.every((p) => $user.permissions.includes(p)))
      );
    } else if (somePerms.length > 0) {
      return Boolean(
        $user.is_superuser ||
          ($user.permissions && somePerms.some((p) => $user.permissions.includes(p)))
      );
    }
    return true;
  });
</script>

{#if $allowed}
  <slot></slot>
{:else}
  <slot name="no-perms"></slot>
{/if}
