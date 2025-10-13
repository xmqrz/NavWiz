<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';
  import { getContext } from 'svelte';
  import { get } from 'svelte/store';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import UserPassEditor from '../../[id=integer]/password/UserPassEditor.svelte';
  import users from '$lib/shared/services/config/users';

  export let data;
  data.user = get(getContext('user'));

  let editor;

  const toastStore = getToastStore();

  function onSave(e) {
    users
      .changeOwnPassword(e.detail)
      .then((d) => {
        toastStore.trigger({
          message: users.successUpdateMsg(d.username),
          timeout: 3000,
          hoverable: true
        });
        editor.muteUnload();
        // Prevent using preloaded data
        goto(data.next ? data.next : users.changeOwnPasswordUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title="Change my password"
  back={data.next ? ['Back', data.next] : []}
  validation={false}>
  <UserPassEditor bind:data on:save={onSave} bind:this={editor} />
</ConfigLayout>
