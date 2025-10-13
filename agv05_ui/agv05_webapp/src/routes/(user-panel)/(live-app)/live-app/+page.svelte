<script>
  import base64 from 'base64-js';
  import cashDom from 'cash-dom';
  import { onMount, getContext } from 'svelte';

  import { amclChannel, mapXChannel as mapChannel, statusChannel } from 'stores/sock/index.js';
  import mapViz from '$lib/shared/mapx/viz';

  const user = getContext('user');
  const mainController = getContext('mainController');
  const moduleManager = mainController.moduleService.getManager();
  const liveApp = mainController.moduleService.getSub('live-app');

  var element;
  var viz;
  var runMode;

  var mapQuality = '0.0';

  const resizeListener = () => {
    vizResize();
  };

  function vizInit() {
    element = cashDom('#live-app-view');

    /* Re-initialize svg element. See comment below. */
    viz = mapViz(element[0], {
      grid: true,
      mapLayout: true,
      laserDownSample: $user.is_agv_panel()
    });
    viz.zoom.zoomFit();

    window.addEventListener('resize', resizeListener);
    window.dispatchEvent(new Event('resize'));
  }

  function vizResize() {
    var p = element.parent();
    var rc = p[0].getBoundingClientRect();
    if (viz) {
      viz.resized(rc.width, rc.height);
    }
  }

  function mapCallback(data) {
    if (data.id === 'laser_pose') {
      viz.scene.updateLaserPose(data.laser_pose);
    } else if (data.id === 'laser_scan') {
      data.laser_scan.ranges = new Float32Array(
        base64.toByteArray(data.laser_scan.ranges).buffer
      );
      viz.scene.updateLaserScan(data.laser_scan);
    } else if (data.id === 'reflectors') {
      viz.scene.updateReflectors(data.reflectors);
    } else if (data.id === 'robot_svg') {
      viz.scene.updateRobotSvg(data.robot_svg);
    } else if (data.id === 'robot_pose') {
      viz.scene.updateRobotPose(data.robot_pose);
    } else if (data.id === 'robot_motion') {
      viz.scene.updateRobotMotion(data.robot_motion);
    } else if (data.id === 'raw_map') {
      viz.scene.updateRawMap(data.raw_map);
    } else if (data.id === 'map_layout') {
      try {
        viz.models.load(JSON.parse(data.map_layout));
      } catch (e) {
        console.log('Fail to load map layout in live app.');
      }
    }
  }

  function amclCallback(data) {
    if (data.id === 'particle_cloud') {
      viz.scene.updateParticleCloud(data.particle_cloud);
    }
  }

  function statusCallback(data) {
    mapQuality = '0.0';
    if (data.id === 'diagnostics') {
      data.diagnostics.status.children.some(function (n1) {
        if (n1.name === 'Controllers') {
          n1.children.some(function (n2) {
            if (n2.name === 'AMCL') {
              n2.children.some(function (n3) {
                if (n3.name === 'Map Quality Score') {
                  mapQuality = n3.values[0].value;
                  return true;
                }
              });
              return true;
            }
          });
          return true;
        }
      });
    }
  }

  function moduleManagerCallback(data) {
    if (data.command === 'start_module') {
      if (data.module_id === 'live-app') {
        liveApp.publish({
          command: 'run_mode'
        });
      }
    }
  }

  function liveAppCallback(data) {
    if (data.command === 'run_mode') {
      runMode = data.run_mode;
    }
  }

  onMount(() => {
    // $log.debug($scope.controllerName, '$ionicView.enter');

    vizInit();

    // subscribe
    moduleManager.subscribe(moduleManagerCallback);
    liveApp.subscribe(liveAppCallback);
    mapChannel.subscribe(mapCallback);
    amclChannel.subscribe(amclCallback);
    statusChannel.subscribe(statusCallback);

    // allow time for subscribe to happen first.
    setTimeout(function () {
      liveApp.startModule();
    }, 100);
    setTimeout(function () {
      mapChannel.publish({
        id: 'retrieve_all'
      });
    }, 500);

    return () => {
      window.removeEventListener('resize', resizeListener);

      moduleManager.unsubscribe(moduleManagerCallback);
      liveApp.unsubscribe(liveAppCallback);
      mapChannel.unsubscribe(mapCallback);
      amclChannel.unsubscribe(amclCallback);
      statusChannel.unsubscribe(statusCallback);
    };
  });
</script>

<div class="flex h-full flex-col">
  <div class="p-4">
    Map Quality Score: <span>{mapQuality}</span> %
    {#if runMode}
      <div class="float-right">
        Mode: <span
          class="badge text-white"
          class:variant-filled-primary={runMode === 'Standalone'}
          class:variant-filled-success={runMode !== 'Standalone'}>
          {runMode}
        </span>
      </div>
    {/if}
  </div>
  <div class="grow">
    <svg
      style="width: 100%; height: 100%;border-radius: 1rem"
      xmlns="http://www.w3.org/2000/svg"
      id="live-app-view"></svg>
  </div>
</div>

<style>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
</style>
