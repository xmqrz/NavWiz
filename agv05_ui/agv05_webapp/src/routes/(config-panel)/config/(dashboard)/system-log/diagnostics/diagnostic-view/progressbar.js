import $ from 'cash-dom';

var progressbarHtml = `
<div class="hidden fixed inset-0 z-50 overflow-auto bg-black bg-opacity-50 flex justify-center items-center">
  <div class="relative p-8 bg-white w-96 rounded-lg shadow-lg">
    <div class="flex items-center">
      <h4 class="text-lg font-semibold"><span class="progress-title-text">Loading...</span> <i class="animate-spin fa fa-refresh ml-2" aria-hidden="true"></i></h4>
    </div>
  </div>
</div>
`;

export default function (outer) {
  var progressbar = $(progressbarHtml);
  outer.append(progressbar[0]);

  var title = progressbar.find('.progress-title-text');

  function show() {
    updateProgress('Loading...');
    return new Promise((resolve) => {
      progressbar.toggleClass('hidden', false);
      resolve();
    });
  }

  function hide() {
    return new Promise((resolve) => {
      progressbar.toggleClass('hidden', true);
      resolve();
    });
  }

  function updateProgress(text) {
    title.text(text);
  }

  return {
    show: show,
    hide: hide,
    updateProgress: updateProgress
  };
}
