/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';

export default function (viz) {
  var $viz = $(viz.node());
  var form = $viz.parent();

  var inputName = form.find('#id_name').on('change', function () {
    viz.models.externalTriggerDirty();
  });
  var inputClearName = form.find('#id_clear_name').on('change', function () {
    viz.models.externalTriggerDirty();
    updateForm();
  });
  var inputReplace = form.find('#id_replace').on('change', updateForm);

  function updateForm() {
    if (inputReplace.is(':checked')) {
      inputClearName.prop('disabled', false);
      inputClearName.parent().removeClass('disabled text-muted');
      if (inputClearName.is(':checked')) {
        inputName.prop('disabled', true).attr('placeholder', '(cleared)').val('');
      } else {
        inputName.prop('disabled', false).attr('placeholder', '(no change)');
      }
    } else {
      inputClearName.prop('disabled', true).prop('checked', false);
      inputClearName.parent().addClass('disabled text-muted');
      inputName.prop('disabled', false).attr('placeholder', '(unnamed)');
    }
  }

  updateForm();
}
