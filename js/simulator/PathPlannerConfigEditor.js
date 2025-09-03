import Car from "../physics/Car.js";

const LOCAL_STORAGE_KEY = 'dash_PathPlannerConfig';

const internalConfig = {
  lattice: {
    numStations: 8,
    numLatitudes: 17,
    stationConnectivity: 3,
    latitudeConnectivity: 7
  },

  roadWidth: 3.7 * 2, // meters

  numDynamicFrames: 20,
  numDynamicSubframes: 4,

  dCurvatureMax: Car.MAX_STEER_SPEED / Car.WHEEL_BASE,
  rearAxleToCenter: -Car.REAR_AXLE_POS
};

const defaultConfig = {
  spatialHorizon: 120, // meters
  centerlineStationInterval: 0.5, // meters

  xyGridCellSize: 0.3, // meters
  slGridCellSize: 0.15, // meters
  gridMargin: 20, // meters
  pathSamplingStep: 1, // meters

  cubicPathPenalty: 0,

  collisionDilationS: Car.HALF_CAR_LENGTH + 2, // meters
  hazardDilationS: 8, // meters
  collisionDilationL: Car.HALF_CAR_WIDTH + 0.5, //meters
  hazardDilationL: 0.5, // meters

  dynamicHazardDilationS: 16,
  dynamicHazardDilationL: 0.5,

  obstacleHazardCost: 200,

  laneCenterLatitude: internalConfig.roadWidth / 4,
  laneShoulderLatitude: internalConfig.roadWidth / 2 * 1.1 - Car.HALF_CAR_WIDTH,
  laneCostSlope: 20, // cost / meter
  lanePreferenceDiscount: 55,

  stationReachDiscount: 400,
  extraTimePenalty: 1000,

  hysteresisDiscount: 50,

  speedLimitPenalty: 200,

  hardAccelerationPenalty: 70,
  hardDecelerationPenalty: 50,

  softLateralAccelerationLimit: 4, // m/s^2
  softLateralAccelerationPenalty: 100,
  linearLateralAccelerationPenalty: 10,

  accelerationChangePenalty: 10
};

const parameterNameMap = {
  'spatialHorizon': '空间视野',
  'centerlineStationInterval': '中心线站点间隔',
  'xyGridCellSize': 'XY网格单元大小',
  'slGridCellSize': 'SL网格单元大小',
  'gridMargin': '网格边距',
  'pathSamplingStep': '路径采样步长',
  'cubicPathPenalty': '三次路径惩罚',
  'collisionDilationS': '碰撞膨胀S',
  'hazardDilationS': '危险膨胀S',
  'collisionDilationL': '碰撞膨胀L',
  'hazardDilationL': '危险膨胀L',
  'dynamicHazardDilationS': '动态危险膨胀S',
  'dynamicHazardDilationL': '动态危险膨胀L',
  'obstacleHazardCost': '障碍物危险成本',
  'laneCenterLatitude': '车道中心纬度',
  'laneShoulderLatitude': '车道边缘纬度',
  'laneCostSlope': '车道成本斜率',
  'lanePreferenceDiscount': '车道偏好折扣',
  'stationReachDiscount': '站点到达折扣',
  'extraTimePenalty': '额外时间惩罚',
  'hysteresisDiscount': '滞后折扣',
  'speedLimitPenalty': '速度限制惩罚',
  'hardAccelerationPenalty': '急加速惩罚',
  'hardDecelerationPenalty': '急减速惩罚',
  'softLateralAccelerationLimit': '软横向加速度限制',
  'softLateralAccelerationPenalty': '软横向加速度惩罚',
  'linearLateralAccelerationPenalty': '线性横向加速度惩罚',
  'accelerationChangePenalty': '加速度变化惩罚'
};

export default class PathPlannerConfigEditor {
  constructor() {
    this._config = Object.assign({}, defaultConfig);

    this.showConfigBox = document.getElementById('show-config-box');
    this.configBox = document.getElementById('config-box-content');
    this.configForm = document.getElementById('config-form');

    this._setUpButtons();

    let storedConfig = {};
    try {
      storedConfig = JSON.parse(window.localStorage.getItem(LOCAL_STORAGE_KEY)) || {};
    } catch (e) {}

    for (const key of Object.keys(this._config).sort()) {
      if (storedConfig[key] !== undefined) this._config[key] = storedConfig[key];
      this.configForm.appendChild(this._createConfigField(key, this._config[key]));
    }
  }

  get config() {
    return Object.assign({}, this._config, internalConfig);
  }

  _setUpButtons() {
    document.getElementById('show-config-button').addEventListener('click', e => {
      this.showConfigBox.classList.add('is-hidden');
      this.configBox.classList.remove('is-hidden');
    });

    document.getElementById('hide-config-button').addEventListener('click', e => {
      this.showConfigBox.classList.remove('is-hidden');
      this.configBox.classList.add('is-hidden');
    });

    document.getElementById('save-config-button').addEventListener('click', this._saveConfigFields.bind(this));
    document.getElementById('restore-defaults-config-button').addEventListener('click', this._restoreDefaults.bind(this));
  }

  _createConfigField(key, value) {
    // 使用中文名称替换英文参数名称
    const displayName = parameterNameMap[key] || key;
    
    const html =
      `<div class="field is-horizontal">
          <div class="field-label is-small" style="flex-grow: 100;">
              <label class="label has-text-grey-light" for="config-field-${key}">${displayName}</label>
          </div>
          <div class="field-body">
              <div class="field">
                  <div class="control" style="margin-right: 16px;">
                      <input id="config-field-${key}" name="${key}" class="input is-small ${value !== defaultConfig[key] ? 'is-danger' : ''}" type="text" style="width: 60px; border-width: 2px;" value="${value}" />
                  </div>
              </div>
          </div>
      </div>`;

    const template = document.createElement('template');
    template.innerHTML = html;
    return template.content.firstChild;
  }

  _saveConfigFields() {
    const formData = new FormData(this.configForm);

    for (const [k, v] of formData.entries()) {
      const parsedValue = Number.parseFloat(v);
      this._config[k] = parsedValue

      const fieldDom = document.getElementById(`config-field-${k}`);
      if (parsedValue === defaultConfig[k])
        fieldDom.classList.remove('is-danger');
      else
        fieldDom.classList.add('is-danger');
    }

    try {
      window.localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(this._config));
    } catch (e) {}
  }

  _restoreDefaults() {
    this._config = Object.assign({}, defaultConfig);

    try {
      window.localStorage.removeItem(LOCAL_STORAGE_KEY);
    } catch (e) {}

    while (this.configForm.firstChild)
      this.configForm.removeChild(this.configForm.firstChild);

    for (const key of Object.keys(this._config).sort())
      this.configForm.appendChild(this._createConfigField(key, this._config[key]));
  }
}

PathPlannerConfigEditor.internalConfig = internalConfig;
