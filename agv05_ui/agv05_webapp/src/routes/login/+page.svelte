<script>
  import { goto } from '$app/navigation';
  import { page } from '$app/stores';
  import { fly } from 'svelte/transition';

  import Login from '../components/Login.svelte';

  let next = $page.url.searchParams.get('next') || '/';
  function handleLogin() {
    goto(next);
  }
</script>

<div out:fly={{ x: -200, duration: 200 }} class="h-full w-full">
  <Login on:login={handleLogin} />
</div>
