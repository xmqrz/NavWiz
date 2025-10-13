import { downloadFileAPI, formAPI, getAPI } from '$lib/utils';

// URLS

function backupRestoreUrl() {
  return '/config/backup-restore';
}

function advancedBackupUrl() {
  return '/config/backup-restore/backup-advanced';
}

function restoreUrl() {
  return '/config/backup-restore/restore';
}

function advancedRestoreUrl() {
  return '/config/backup-restore/restore-advanced';
}

// MSGS
function successRestoreMsg() {
  return 'Configuration restored.';
}

// REST API
function checkLicenseVoid(fetch = undefined) {
  return getAPI('/config/backup', fetch);
}

function backup(data) {
  return downloadFileAPI('/config/backup/download', data, 'POST');
}

function advancedBackup(data) {
  return downloadFileAPI('/config/backup/advanced-download', data, 'POST');
}

function backupAdvancedOptions(fetch = undefined) {
  return getAPI('/config/backup/advanced-download', fetch);
}

function restore(form, fetch = undefined) {
  return formAPI('/config/backup/restore', form, 'POST', fetch);
}

function advancedRestore(form, fetch = undefined) {
  return formAPI('/config/backup/advanced-restore', form, 'POST', fetch);
}

function getRestoreChoices(form, fetch = undefined) {
  return formAPI('/config/backup/restore', form, 'PUT', fetch);
}

function getRestoreOptionsTree(form, fetch = undefined) {
  return formAPI('/config/backup/advanced-restore', form, 'PUT', fetch);
}

export default {
  checkLicenseVoid,
  backupRestoreUrl,
  advancedBackupUrl,
  restoreUrl,
  advancedRestoreUrl,

  successRestoreMsg,

  backup,
  advancedBackup,
  backupAdvancedOptions,
  restore,
  advancedRestore,
  getRestoreChoices,
  getRestoreOptionsTree
};
