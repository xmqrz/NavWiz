<script>
  import base64 from 'base64-js';
  import { getContext } from 'svelte';

  import { amclChannel, mapXChannel as mapChannel } from 'stores/sock/index.js';
  import mapViz from '$lib/shared/mapx/viz';

  const user = getContext('user');

  var element;
  var viz;
  var lastCostmapTime = new Date();

  function vizInit(dom) {
    element = dom;
    viz = mapViz(dom, {
      grid: true,
      mapLayout: true,
      laserDownSample: $user.is_agv_panel()
    });
    viz.zoom.zoomFit();

    window.addEventListener('resize', vizResize);
    vizResize();

    mapChannel.subscribe(mapCallback);
    amclChannel.subscribe(amclCallback);

    // allow time for subscriber to happen first.
    setTimeout(function () {
      mapChannel.publish({
        id: 'retrieve_all'
      });
      amclChannel.publish({
        id: 'retrieve_all'
      });
    }, 100);
    return {
      destroy() {
        window.removeEventListener('resize', vizResize);
        mapChannel.unsubscribe(mapCallback);
        amclChannel.unsubscribe(amclCallback);
      }
    };
  }

  function vizResize() {
    var p = element.parentNode;
    var rc = p.getBoundingClientRect();
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
      if (data.laser_scan.intensities) {
        data.laser_scan.intensities = new Float32Array(
          base64.toByteArray(data.laser_scan.intensities).buffer
        );
      }
      viz.scene.updateLaserScan(data.laser_scan);
    } else if (data.id === 'reflectors') {
      viz.scene.updateReflectors(data.reflectors);
    } else if (data.id === 'docking_reflectors') {
      viz.scene.updateDockingReflectors(data.docking_reflectors);
    } else if (data.id === 'docking_cloud') {
      data.docking_cloud.cloud = new Float32Array(
        base64.toByteArray(data.docking_cloud.cloud).buffer
      );
      viz.scene.updateDockingCloud(data.docking_cloud);
    } else if (data.id === 'target_pose') {
      viz.scene.updateMarkerPose(data.marker_pose);
      viz.scene.updateTargetPose(data.target_pose);
    } else if (data.id === 'docking_plan') {
      viz.scene.updateDockingPlan(data.docking_plan);
    } else if (data.id === 'robot_svg') {
      viz.scene.updateRobotSvg(data.robot_svg);
    } else if (data.id === 'robot_pose') {
      viz.scene.updateRobotPose(data.robot_pose);
    } else if (data.id === 'robot_path') {
      viz.scene.updateRobotPath(data.robot_path);
    } else if (data.id === 'robot_motion') {
      viz.scene.updateRobotMotion(data.robot_motion);
    } else if (data.id === 'raw_map') {
      viz.scene.updateRawMap(data.raw_map);
    } else if (data.id === 'costmap') {
      viz.scene.updateCostmap(data.costmap);
      lastCostmapTime = new Date();
    } else if (data.id === 'map_layout') {
      try {
        viz.models.load(JSON.parse(data.map_layout));
      } catch (e) {
        console.log('Fail to parse map layout.');
      }
    }

    if (new Date() - lastCostmapTime > 3000) {
      // clear stale costmap (>3s)
      viz.scene.clearCostmap();
    }
  }

  function amclCallback(data) {
    if (data.id === 'particle_cloud') {
      viz.scene.updateParticleCloud(data.particle_cloud);
    }
  }
</script>

<div class="h-full w-full">
  <svg
    style="width: 100%; height: 100%;border-radius: 1rem"
    xmlns="http://www.w3.org/2000/svg"
    id="mapx-live-view"
    use:vizInit></svg>
</div>

<style global>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
</style>
