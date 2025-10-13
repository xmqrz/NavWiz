import $ from 'cash-dom';

import AR from './ar.js';
import Barhead from './barhead.js';
import Bartail from './bartail.js';
import BarReflector from './barreflector.js';
import BarAR from './barar.js';
import L2Marker from './l2marker.js';
import LMarker from './lmarker.js';
import Pallet from './pallet.js';
import Reflector from './reflector.js';
import V2Marker from './v2marker.js';
import VLMarker from './vlmarker.js';
import VMarker from './vmarker.js';

var markerHtml = (keyboard) => `
<div class="absolute inset-0 z-50 min-h-screen overflow-y-auto scrollbar-hide hidden backdrop-blur-sm backdrop-contrast-50" tabindex="-1" role="dialog" aria-labelledby="marker-type-configuration-modal-label">
  <div class="flex items-center justify-center min-h-screen" role="document">
    <div class="bg-white rounded-2xl shadow-lg min-w-96">
      <div class="flex justify-between items-center px-4 py-2 border-b bg-secondary-400-500-token rounded-t-2xl">
        <h4 class="text-lg font-semibold" id="marker-type-configuration-modal-label">Marker Type Configuration</h4>
        <button type="button" class="variant-ringed-surface bg-secondary-400-500-token btn btn-sm w-10 rounded ring-white marker-config-close" aria-label="Close">
          <i class="fa fa-close"></i>
        </button>
      </div>
      <div class="min-w-28 px-4 py-4">
        <div class="flex items-center justify-around mb-4" style="height: 80px;">
          <div id="marker-type-display" style="height:80px;">
          </div>
          <canvas id="angleRangeCanvas" width="200px" height="80px"></canvas>
        </div>
        <form id="marker-type-configuration-form">
          <div class="grid grid-cols-2 gap-2 items-center">
            <label for="marker-type-selection">Marker Type:</label>
            <select class="select" id="marker-type-selection" required>
              <option value="reflector">Reflector</option>
              <option value="vmarker">V-Marker</option>
              <option value="lmarker">L-Marker</option>
              <option value="vlmarker">VL-Marker</option>
              <option value="v2marker">V-Pair-Marker</option>
              <option value="l2marker">L-Pair-Marker</option>
              <option value="barhead">Bar-Head-Marker</option>
              <option value="bartail">Bar-Tail-Marker</option>
              <option value="barreflector">Bar-Reflector-Marker</option>
              <option value="barar">Bar-AR-Marker</option>
              <option value="ar">AR</option>
              <option value="pallet">Pallet</option>
            </select>

            <label class="profileOptions" for="marker-param-profile">Marker Localization Profile:</label>
            <select class="select form-control profileOptions" id="marker-param-profile">
              <option value="1">Profile 1</option>
              <option value="2">Profile 2</option>
              <option value="3">Profile 3</option>
              <option value="4">Profile 4</option>
              <option value="5">Profile 5</option>
            </select>

            <label class="sensorOptions" for="marker-param-sensor">Sensor:</label>
            <select class="select form-control sensorOptions" id="marker-param-sensor" required>
              <option value="laser1">Laser 1</option>
              <option value="laser2">Laser 2</option>
              <option value="laser3">Laser 3</option>
              <option value="laser4">Laser 4</option>
              <option value="laser5">Laser 5</option>
              <option value="laser6">Laser 6</option>
            </select>

            <label class="additionalSensorOptions hidden" for="marker-param-additional-sensor">Additional Sensor:</label>
            <select class="select form-control additionalSensorOptions hidden" id="marker-param-additional-sensor">
              <option value="laser1">Laser 1</option>
              <option value="laser2">Laser 2</option>
              <option value="laser3">Laser 3</option>
              <option value="laser4">Laser 4</option>
              <option value="laser5">Laser 5</option>
              <option value="laser6">Laser 6</option>
            </select>

            <label class="angleConfiguration" for="marker-param-sensor-inverted">Sensor Inversely Mounted:</label>
            <input type="checkbox" class="checkbox form-control angleConfiguration" id="marker-param-sensor-inverted">
            <label class="angleConfiguration" for="marker-param-angle-min">Min Angle (degree):</label>
            <input type="number" class="input angleConfiguration ${keyboard ? 'keyboard' : ''}" id="marker-param-angle-min" min=-360 max=360>
            <label class="angleConfiguration" for="marker-param-angle-max">Max Angle (degree):</label>
            <input type="number" class="input angleConfiguration ${keyboard ? 'keyboard' : ''}" id="marker-param-angle-max" min=-360 max=360>

            <label class="rangeMaxConfiguration" for="marker-param-range-max">Maximum sensing range (m):</label>
            <input type="number" class="input rangeMaxConfiguration ${keyboard ? 'keyboard' : ''}" id="marker-param-range-max" min=0 max=50>

            <label class="idConfiguration1 hidden" for="marker-param-marker-id1">ArUco Marker ID 1:</label>
            <input type="number" class="input idConfiguration1 hidden ${keyboard ? 'keyboard' : ''}" id="marker-param-marker-id1" min=0 max=1023>

            <label class="idConfiguration2 hidden" for="marker-param-marker-id2">ArUco Marker ID 2:</label>
            <input type="number" class="input idConfiguration2 hidden ${keyboard ? 'keyboard' : ''}" id="marker-param-marker-id2" min=0 max=1023>

            <p>Marker Type: </p>
            <p id="marker-type-preview">reflector1__laser1</>
          </div>
        </form>
      </div>
      <div class="flex justify-end px-4 py-2 border-t">
        <button type="button" class="marker-config-close btn mr-2 variant-ringed-surface">Close</button>
        <button type="button" class="btn variant-ringed-surface" id="marker-config-apply">Apply</button>
      </div>
    </div>
  </div>
</div>
`;

export default function (parent, keyboard) {
  var marker = $(markerHtml(keyboard));

  parent.append(marker);

  var markerTypeConfigurationModalShown = false;
  var _marker_type = new Reflector();
  const markerTypes = [
    'reflector',
    'vmarker',
    'v2marker',
    'vlmarker',
    'lmarker',
    'l2marker',
    'barhead',
    'bartail',
    'barreflector',
    'barar',
    'ar',
    'pallet'
  ];

  var markerTypePreview = marker.find('#marker-type-preview');
  var markerTypeDisplay = marker.find('#marker-type-display');
  var markerProfileSelection = marker
    .find('#marker-param-profile')
    .on('change', updateMarkerTypePreview);
  var markerSensorSelection = marker
    .find('#marker-param-sensor')
    .on('change', updateMarkerTypePreview);
  var additionalMarkerSensorSelection = marker
    .find('#marker-param-additional-sensor')
    .on('change', updateMarkerTypePreview);
  var markerSensorInverted = marker
    .find('#marker-param-sensor-inverted')
    .on('change', updateMarkerTypePreview);
  var markerAngleMin = marker
    .find('#marker-param-angle-min')
    .on('change', updateMarkerTypePreview);
  var markerAngleMax = marker
    .find('#marker-param-angle-max')
    .on('change', updateMarkerTypePreview);
  var markerRangeMax = marker
    .find('#marker-param-range-max')
    .on('change', updateMarkerTypePreview);
  var markerId1 = marker
    .find('#marker-param-marker-id1')
    .on('change', updateMarkerTypePreview);
  var markerId2 = marker
    .find('#marker-param-marker-id2')
    .on('change', updateMarkerTypePreview);
  marker.find('.marker-config-close').on('click', function () {
    hide();
  });
  var angleRangeCanvas = marker.find('#angleRangeCanvas')[0];
  let angleRangeContext = angleRangeCanvas.getContext('2d');
  angleRangeContext.fillStyle = 'rgba(216, 64, 64, 0.7)';
  var markerTypeConfigurationForm = marker.find('#marker-type-configuration-form');
  marker.find('#marker-config-apply').on('click', function () {
    if (markerTypeConfigurationForm[0].reportValidity()) {
      marker.trigger('apply', markerTypePreview.text());
      hide();
    }
  });
  var markerTypeSelection = marker.find('#marker-type-selection').on('change', function () {
    updateMarkerType($(this).val());
    updateMarkerTypeDisplay();
    updateMarkerModalOptions($(this).val());
    updateMarkerTypePreview();
  });

  function showMarkerTypeConfigurationModal(initValue) {
    resetAngleRangeCanvas();
    marker.show();
    markerTypeConfigurationModalShown = true;
    marker.trigger('visibility');
    // Extract data and set to default if invalid
    if (initValue) {
      let [markerPart, sensorPart, anglePart] = initValue.split('__');
      let match = markerPart.match(/^(\D+?)(\d*)$/);
      let markerType = match ? match[1] : 'reflector';
      if (!markerTypes.includes(markerType)) {
        markerType = 'reflector';
      }
      updateMarkerType(markerType);
      updateMarkerModalOptions();
      markerTypeSelection.val(markerType);

      if (_marker_type.allowProfileConfiguration) {
        let profile = match && match[2] !== '' ? match[2] : '1';
        markerProfileSelection.val(profile);
      }

      let [sensor, additionalSensor] = sensorPart ? sensorPart.split('_') : ['', ''];
      if (!_marker_type.sensorOptions.map((sensor) => sensor.value).includes(sensor)) {
        sensor = _marker_type.sensorOptions[0].value;
      }
      markerSensorSelection.val(sensor);

      if (_marker_type.additonalSensor) {
        if (
          !_marker_type.additionalSensorOptions
            .map((additionalSensor) => additionalSensor.value)
            .includes(additionalSensor)
        ) {
          additionalSensor = _marker_type.additionalSensorOptions[0].value;
        }
        additionalMarkerSensorSelection.val(additionalSensor);
      }

      if (_marker_type.allowAngleConfiguration) {
        let [minAngle, maxAngle] = anglePart ? anglePart.split('_') : ['', ''];
        if (minAngle !== '' && maxAngle !== '') {
          minAngle = Math.max(_marker_type.minAngle, minAngle);
          maxAngle = Math.min(_marker_type.maxAngle, maxAngle);
          markerAngleMin.val(minAngle);
          markerAngleMax.val(maxAngle);
        }
      } else if (_marker_type.allowIdConfiguration) {
        // anglePart here is Marker Id for AR Marker
        let [firstId, secondId] = anglePart ? anglePart.split('_') : ['', ''];
        markerId1.val(firstId);
        if (_marker_type.singleMarkerOnly) {
          markerId2.val('');
        } else {
          markerId2.val(secondId);
        }
      }

      if (_marker_type.allowRangeMaxConfiguration) {
        let rmaxPart = initValue.split('__r');
        let rmax = rmaxPart[1] ? rmaxPart[1] : '';
        markerRangeMax.val(rmax);
      }
    }
    updateMarkerTypeDisplay();
    updateMarkerTypePreview();
  }

  function hide() {
    marker.hide();
    markerTypeConfigurationModalShown = false;
    marker.trigger('visibility');
  }

  function resetMarkerModalValuesAndDisplay() {
    markerSensorSelection.find('option').remove();
    additionalMarkerSensorSelection.find('option').remove();
    $('.profileOptions').find('option').remove();
    markerSensorInverted.prop('checked', false);
    markerAngleMin.val('');
    markerAngleMax.val('');
    markerRangeMax.val('');
    markerId1.val('');
    markerId2.val('');
    $('.profileOptions').hide();
    $('.angleConfiguration').hide();
    $('.rangeMaxConfiguration').hide();
    $('.idConfiguration1').hide();
    $('.idConfiguration2').hide();
    $('.additionalSensorOptions').hide();
    $('#angleRangeCanvas').hide();
    resetAngleRangeCanvas();
  }

  function resetAngleRangeCanvas() {
    angleRangeContext.clearRect(0, 0, angleRangeCanvas.width, angleRangeCanvas.height);
    angleRangeContext.beginPath();
    let minRadOffset = (_marker_type.minAngle * Math.PI) / 180 - Math.PI / 2;
    let maxRadOffset = (_marker_type.maxAngle * Math.PI) / 180 - Math.PI / 2;
    angleRangeContext.arc(100, 40, 40, minRadOffset, maxRadOffset);
    angleRangeContext.fill();
  }

  function updateMarkerType(markerType) {
    switch (markerType) {
      case 'vmarker':
        _marker_type = new VMarker();
        break;
      case 'lmarker':
        _marker_type = new LMarker();
        break;
      case 'vlmarker':
        _marker_type = new VLMarker();
        break;
      case 'v2marker':
        _marker_type = new V2Marker();
        break;
      case 'l2marker':
        _marker_type = new L2Marker();
        break;
      case 'barhead':
        _marker_type = new Barhead();
        break;
      case 'bartail':
        _marker_type = new Bartail();
        break;
      case 'barreflector':
        _marker_type = new BarReflector();
        break;
      case 'barar':
        _marker_type = new BarAR();
        break;
      case 'ar':
        _marker_type = new AR();
        break;
      case 'pallet':
        _marker_type = new Pallet();
        break;
      case 'reflector':
      default:
        _marker_type = new Reflector();
        break;
    }
  }

  function updateMarkerTypeDisplay() {
    markerTypeDisplay.empty();
    markerTypeDisplay.append(_marker_type.svg);
  }

  function updateMarkerModalOptions() {
    resetMarkerModalValuesAndDisplay();
    for (let sensor of _marker_type.sensorOptions) {
      markerSensorSelection.append(
        `<option value=` + sensor.value + `>` + sensor.display + `</option>`
      );
    }
    if (_marker_type.additonalSensor) {
      for (let sensor of _marker_type.additionalSensorOptions) {
        additionalMarkerSensorSelection.append(
          `<option value=` + sensor.value + `>` + sensor.display + `</option>`
        );
      }
      $('.additionalSensorOptions').show();
    }
    for (let profile of _marker_type.profileOptions) {
      markerProfileSelection.append(
        `<option value=` + profile.value + `>` + profile.display + `</option>`
      );
    }
    if (_marker_type.allowProfileConfiguration) {
      $('.profileOptions').show();
    }
    if (_marker_type.allowAngleConfiguration) {
      $('.angleConfiguration').show();
      $('#angleRangeCanvas').show();
    }
    if (_marker_type.allowRangeMaxConfiguration) {
      $('.rangeMaxConfiguration').show();
    }
    if (_marker_type.allowIdConfiguration) {
      $('.idConfiguration1').show();
      if (!_marker_type.singleMarkerOnly) {
        $('.idConfiguration2').show();
      }
    }
  }

  function updateMarkerTypePreview() {
    if (markerTypeConfigurationForm[0].reportValidity()) {
      markerTypePreview.text('');
      let preview = markerTypeSelection.val();
      if (markerProfileSelection.val()) {
        preview = preview.concat(markerProfileSelection.val());
      }
      preview = preview.concat('__').concat(markerSensorSelection.val());
      if (_marker_type.additonalSensor && additionalMarkerSensorSelection.val()) {
        preview = preview.concat('_').concat(additionalMarkerSensorSelection.val());
      }
      if (markerAngleMin.val() !== '' && markerAngleMax.val() !== '') {
        let minAngle = parseInt(markerAngleMin.val(), 10) || 0;
        let maxAngle = parseInt(markerAngleMax.val(), 10) || 0;
        if (maxAngle > minAngle) {
          minAngle = Math.max(_marker_type.minAngle, minAngle);
          maxAngle = Math.min(_marker_type.maxAngle, maxAngle);
          markerAngleMin.val(minAngle);
          markerAngleMax.val(maxAngle);
          preview = preview.concat('__').concat(minAngle).concat('_').concat(maxAngle);
          let minRadOffset =
            ((markerSensorInverted.is(':checked') ? minAngle : -maxAngle) * Math.PI) / 180 -
            Math.PI / 2;
          let maxRadOffset =
            ((markerSensorInverted.is(':checked') ? maxAngle : -minAngle) * Math.PI) / 180 -
            Math.PI / 2;
          angleRangeContext.clearRect(0, 0, angleRangeCanvas.width, angleRangeCanvas.height);
          angleRangeContext.beginPath();
          angleRangeContext.arc(100, 40, 40, minRadOffset, maxRadOffset);
          angleRangeContext.lineTo(100, 40);
          angleRangeContext.fill();
        } else {
          resetAngleRangeCanvas();
        }
      }
      if (_marker_type.allowIdConfiguration) {
        const id1Str = markerId1.val();
        const id2Str = markerId2.val();
        const hasId1 = id1Str !== '';
        const hasId2 = id2Str !== '';
        if (hasId1) {
          const id1 = parseInt(id1Str, 10) || 0;
          preview = preview.concat('__').concat(id1);

          if (!_marker_type.singleMarkerOnly && hasId2) {
            const id2 = parseInt(id2Str, 10) || 0;
            preview = preview.concat('_').concat(id2);
          }
        }
      }
      if (markerRangeMax.val() !== '') {
        let maxRange = parseInt(markerRangeMax.val(), 10) || 0;
        preview = preview.concat('__r').concat(maxRange);
      }
      markerTypePreview.text(preview);
    }
  }

  return {
    show: showMarkerTypeConfigurationModal,
    hide: hide,
    on: marker.on.bind(marker),
    visible: () => markerTypeConfigurationModalShown
  };
}
