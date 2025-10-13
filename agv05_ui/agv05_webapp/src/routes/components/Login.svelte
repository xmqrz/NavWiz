<script>
  import { createEventDispatcher } from 'svelte';
  import logo from '$lib/assets/img/navwiz_logo.svg';
  import background from '$lib/assets/img/navwiz_login_bg.jpg';
  import { login } from 'stores/auth.js';

  const dispatch = createEventDispatcher();

  let username = '';
  let password = '';
  let error = '';
  async function handleSubmit() {
    const err = await login(username, password);
    if (err) {
      console.log(err);
      error = err;
      return;
    }

    username = '';
    password = '';
    error = '';
    dispatch('login');
  }
</script>

<div class="relative flex h-full w-full flex-row-reverse overflow-hidden">
  <div class="absolute h-full w-full">
    <img
      src={background}
      class="float-right min-h-full min-w-full max-w-none"
      alt="NavWiz Background" />
  </div>
  <div
    class="relative flex h-full w-full min-w-[600px] flex-col items-center justify-center space-y-10 bg-primary-200 bg-opacity-60 shadow-2xl backdrop-blur-sm md:w-6/12">
    <img src={logo} class="w-60 bg-white shadow-2xl" alt="NavWiz Logo" />
    <h1 class="text-4xl font-thin tracking-[0.2em]">LOGIN</h1>
    <div class="card w-full max-w-md bg-gray-50 p-10 shadow-2xl ring-0">
      <form on:submit|preventDefault={handleSubmit}>
        <label class="label pb-7">
          <span class="tracking-widest text-gray-800">User Name</span>
          <input
            class="input p-2 px-4"
            type="text"
            placeholder="User Name"
            bind:value={username} />
        </label>
        <label class="label pb-3">
          <span class="tracking-widest text-gray-800">Password</span>
          <input
            class="input p-2 px-4"
            type="password"
            placeholder="Password"
            bind:value={password} />
        </label>
        <div class:invisible={!error} class="text-error-600-300-token pb-4 text-center">
          {error || '.'}
        </div>
        <input class="variant-filled-primary btn mt-4 w-full" type="submit" value="Login" />
      </form>
    </div>
  </div>
</div>
