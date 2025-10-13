/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import DiagnosticReader from './diagnostic-reader';
import perf from '$lib/shared/perf';

export default function (viz) {
  var diagnosticReader = new DiagnosticReader();
  var filename = '';

  function load(options) {
    perf.begin('load init');
    filename = options.filename || 'data.csv';
    return diagnosticReader.init(options.blob).then(function () {
      perf.end('load init');
      viz.trigger('models.loaded');
    });
  }

  viz.models = {
    load: load,

    rawFilename: () => filename,

    timestamps: diagnosticReader.timestamps,
    headers: diagnosticReader.headers,
    startTime: diagnosticReader.startTime,
    getData: diagnosticReader.getData
  };
}
