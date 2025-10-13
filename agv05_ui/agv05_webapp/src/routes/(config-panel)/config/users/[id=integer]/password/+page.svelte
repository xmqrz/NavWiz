<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import UserPassEditor from './UserPassEditor.svelte';
  import users from '$lib/shared/services/config/users';

  export let data;
  let editor;

  const toastStore = getToastStore();

  function onSave(e) {
    users
      .changePassword(data.user.id, e.detail)
      .then((d) => {
        toastStore.trigger({
          message: users.successUpdateMsg(d.username),
          timeout: 3000,
          hoverable: true
        });
        editor.muteUnload();
        // Prevent using preloaded data
        // TODO: why goto invalidateAll option not working.
        goto(users.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title={`Change password of user "${data.user.username}"`}
  back={['Users', users.listUrl()]}
  validation={false}>
  <UserPassEditor {data} on:save={onSave} bind:this={editor} />
</ConfigLayout>
