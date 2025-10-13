import $ from 'node_modules/cash-dom';

function jquery(selection, context) {
  // Monkey patch to avoid template with extra tag similar to jquery.
  if (typeof selection == 'string' && context == undefined) {
    selection = selection.trim();
  }
  return $(selection, context);
}

jquery.prototype = $.prototype;

export default jquery;
