<script>
  import FormTab from 'components/FormTab.svelte';
  import ClearableFileInput from './ClearableFileInput.svelte';
  import Select from 'components/Select.svelte';

  export let properties;
  export let initial;
  export let layout;

  let groups = Object.entries(layout.groups);
  let parameters = layout.parameters.map(getProp);

  function getProp(fieldName) {
    let prop = properties[fieldName];
    prop.name = fieldName;

    // patch for multiple file upload
    if (prop.type === 'list' && prop.child.type === 'file upload') {
      prop = Object.assign(prop, prop.child);
      prop.type = 'multi file upload';
    }

    return prop;
  }

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';

  // TODO: indicator if a field is dirty would be nice to have. when user submit it return to not dirty.
</script>

{#key initial}
  <div class="grid grid-cols-4">
    <div class="col-span-4">
      {#each parameters as param}
        {@const value = initial[param.name]}
        <div class="p-3 px-10 {param.label.trim() ? '' : 'pt-0'} grid grid-cols-4">
          {#if param.type === 'checkbox multiple choice'}
            <!-- NOTE: separate from others field to prevent label in label issue -->
            <div class={formLabelClass}>{param.label}</div>
            <div class="col-span-3">
              {#each param.choices as choice}
                <label class="label mr-5 inline-block">
                  <input
                    name={param.name}
                    class="checkbox mr-1"
                    type="checkbox"
                    value={choice.value}
                    checked={value.includes(choice.value)}
                    disabled={param.read_only} />
                  {choice.display_name}
                </label>
              {/each}
            </div>
          {:else}
            <label class="col-span-4 grid grid-cols-4">
              {#if param.type === 'boolean'}
                <div class="col-span-3 col-start-2 flex items-center space-x-2">
                  <input
                    name={param.name}
                    class="checkbox mr-1"
                    type="checkbox"
                    checked={value}
                    disabled={param.read_only} />
                  <span class={formLabelClass}>{param.label}</span>
                </div>
              {:else if param.type === 'checkbox multiple choice'}
                <!-- Note: put outside to prevent label in label-->
              {:else}
                <span class={formLabelClass}>{param.label}</span>
                <div class="col-span-3">
                  {#if param.type === 'choice'}
                    <select
                      class="select px-7 rounded-token"
                      name={param.name}
                      {value}
                      disabled={param.read_only}>
                      {#each param.choices as c}
                        <option value={c.value}>{c.display_name}</option>
                      {/each}
                    </select>
                  {:else if param.type === 'multiple choice'}
                    <div class="w-full">
                      <Select
                        name={param.name}
                        multiple={true}
                        startEmpty={true}
                        enableFilter={false}
                        buttonClass="w-full px-7 mx-0 variant-form rounded-token"
                        class="w-[70%]"
                        options={param.choices.map((c) => [c.value, c.display_name])}
                        {value}
                        disabled={param.read_only} />
                    </div>
                  {:else if param.type === 'file upload'}
                    <ClearableFileInput
                      accept={param.accept}
                      name={param.name}
                      {value}
                      disabled={param.read_only} />
                  {:else if param.type === 'multi file upload'}
                    <ClearableFileInput
                      accept={param.accept}
                      name={param.name}
                      multiple={true}
                      {value}
                      disabled={param.read_only} />
                  {:else if param.type === 'char' || param.type === 'string'}
                    <input
                      name={param.name}
                      class="input px-7"
                      type="text"
                      {value}
                      required={!param.read_only}
                      disabled={param.read_only} />
                  {:else if param.type === 'textarea string'}
                    <textarea
                      name={param.name}
                      rows="10"
                      cols="40"
                      class="textarea"
                      required={!param.read_only}
                      disabled={param.read_only}>{value}</textarea>
                  {:else if param.type === 'float'}
                    <input
                      name={param.name}
                      class="input px-7"
                      type="number"
                      step="any"
                      min={param.min_value}
                      max={param.max_value}
                      {value}
                      required={!param.read_only}
                      disabled={param.read_only} />
                  {:else if param.type === 'integer'}
                    <input
                      name={param.name}
                      class="input px-7"
                      type="number"
                      min={param.min_value}
                      max={param.max_value}
                      {value}
                      required={!param.read_only}
                      disabled={param.read_only} />
                  {/if}
                </div>
              {/if}
            </label>
          {/if}
          {#if param.help_text}
            <!-- eslint-disable-next-line svelte/no-at-html-tags -->
            <span class="col-span-3 col-start-2 text-gray-500">{@html param.help_text}</span>
          {/if}
        </div>
      {/each}
    </div>
  </div>
{/key}

{#if groups.length > 0}
  <FormTab items={groups} classAttr="pt-3">
    <div slot="panel" let:data={value}>
      <svelte:self {properties} {initial} layout={value} />
    </div>
  </FormTab>
{/if}
