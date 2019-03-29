function TestStabilizeLiquid() {
  camera.position.y = 1;
  camera.position.z = 4;

  var bd = new b2BodyDef();
  var ground = world.CreateBody(bd);

  this.w = 2.0;
  var gen_cup = function (x, y) {
    var bd = new b2BodyDef();
    bd.type = b2_dynamicBody;
    bd.allowSleep = false;
    bd.position.Set(x, y);
    var body = world.CreateBody(bd);

    var b1 = new b2PolygonShape();
    b1.SetAsBoxXYCenterAngle(0.05, 0.5, new b2Vec2(0.5, 0), 0);
    body.CreateFixtureFromShape(b1, 5);

    var b2 = new b2PolygonShape();
    b2.SetAsBoxXYCenterAngle(0.05, 0.5, new b2Vec2(-0.5, 0), 0);
    body.CreateFixtureFromShape(b2, 5);

    var b4 = new b2PolygonShape();
    b4.SetAsBoxXYCenterAngle(0.5, 0.05, new b2Vec2(0, -0.5), 0);
    body.CreateFixtureFromShape(b4, 5);
    var base = world.CreateBody(bd);

    var jdb = new b2RevoluteJointDef();
    jdb.motorSpeed = 0 * Math.PI;
    jdb.maxMotorTorque = 1e7;
    jdb.enableMotor = true;
    var rot = jdb.InitializeAndCreate(base, body, new b2Vec2(x, y));

    var jd = new b2PrismaticJointDef();
    jd.lowerTranslation = -5;
    jd.upperTranslation = 5;
    jd.enableLimit = true;
    jd.maxMotorForce = 1e7;
    jd.motorSpeed = 0;
    jd.enableMotor = true;
    var trans = jd.InitializeAndCreate(ground, base, base.GetWorldCenter(), new b2Vec2(1.0, 0.0));
    return {
      body: body,
      base: base,
      rot: rot,
      trans: trans
    };
  };

  this.cup1 = gen_cup(-this.w, 2.0);
  this.cup2 = gen_cup(-this.w, -0.5);




  var psd = new b2ParticleSystemDef();
  psd.radius = 0.025;
  psd.dampingStrength = 0.02;

  var particleSystem = world.CreateParticleSystem(psd);
  var gen_water = function (x, y) {
    var box = new b2PolygonShape();
    box.SetAsBoxXYCenterAngle(0.5, 0.5, new b2Vec2(x, y), 0);
    var particleGroupDef = new b2ParticleGroupDef();
    particleGroupDef.shape = box;
    particleGroupDef.flags = b2_waterParticle;
    particleGroupDef.color = particleColors[2];
    var particleGroup = particleSystem.CreateParticleGroup(particleGroupDef);
  };

  gen_water(-this.w, 2.0);
  gen_water(-this.w, -0.5);

  this.time = 0;
  this.r = -this.w;

  /*
   目標値整形フィルタの係数．共振周波数付近の成分を逓減すれば
   角度制御無しでもあまりこぼれない気がするが，
   普通のLPFにして角度制御の効果を観ることにする．
  */

  this.f = {
    A: [
      [9.561453e-01, 2.590687e-01, -2.704004e-03, -6.167686e-03, -1.193954e-03],
      [-2.609884e-01, 6.379289e-01, 1.838834e-01, -5.026902e-02, -1.369106e-02],
      [2.866842e-03, 1.861339e-01, 8.727564e-01, 5.858915e-02, 1.425724e-02],
      [-5.569806e-03, 5.242819e-02, -6.232137e-02, 9.664512e-01, -1.679912e-02],
      [1.376805e-03, -1.504614e-02, 1.521865e-02, 2.111645e-02, 9.650828e-01]
    ],
    B: [
      [4.582305e+00],
      [1.729938e+01],
      [-6.626085e+00],
      [-8.941701e-01],
      [3.120392e-01]
    ],
    C: [
      [-7.102762e-04, 1.163926e-04, -5.796521e-04, 1.474387e-02, 3.391929e-02],
      [-2.619001e-03, -3.968858e-03, -1.803889e-02, 1.274521e-02, -8.779295e-02],
      [4.953646e-02, -4.005582e-02, -3.953558e-02, -1.919667e-01, 1.603253e-01],
      [6.189164e-01, 3.318854e-01, 7.584440e-01, 5.501393e-01, -1.480721e-01],
      [-4.473304e+00, 1.730990e+01, -6.618614e+00, 8.429386e-01, 2.849460e-01]
    ],
    x: [
      [0],
      [0],
      [0],
      [0],
      [0]
    ],
    y: [
      [0],
      [0],
      [0]
    ]
  };
  // 左端からスタートする
  this.f.x = numeric.dot(numeric.inv(this.f.C), [
    [-this.w],
    [0],
    [0],
    [0],
    [0]
  ]);
  this.manual = false;
}

TestStabilizeLiquid.prototype.MouseMove = function (p) {
  this.r = p.x;
  if (this.r > this.w) this.r = this.w;
  if (this.r < -this.w) this.r = -this.w;
};

TestStabilizeLiquid.prototype.MouseUp = function (p) {
  this.manual = false;
};

TestStabilizeLiquid.prototype.MouseDown = function (p) {
  this.manual = true;
};


TestStabilizeLiquid.prototype.Step = function () {
  world.Step(timeStep, velocityIterations, positionIterations);
  this.time += 1 / 60;
  var phase = this.time / 2 * 2 * Math.PI - 4;
  if (phase < 0) phase = 0;
  var r = -this.w * (Math.cos(phase) > 0 ? 1 : -1);
  //r= -this.w*Math.cos(phase);
  if (this.manual) {
    r = this.r;
  }
  // 目標値整形フィルタ
  this.f.x = numeric.add(numeric.dot(this.f.A, this.f.x), numeric.dot([
    [r, 0, 0, 0, 0],
    [0, r, 0, 0, 0],
    [0, 0, r, 0, 0],
    [0, 0, 0, r, 0],
    [0, 0, 0, 0, r]
  ], this.f.B));
  this.f.y = numeric.dot(this.f.C, this.f.x);
  // カップの位置をPD制御
  var u = 100 * (this.f.y[0] - this.cup1.body.GetPosition().x) + 100 * (this.f.y[1] - this.cup1.body.GetLinearVelocity().x);
  this.cup1.trans.SetMotorSpeed(this.cup1.trans.GetMotorSpeed() + u / 60);
  // 慣性力を重力で相殺するようカップ角度の目標値を計算
  var ra = -Math.atan2(this.f.y[2], 10);
  var dra = 1 / (1 + (this.f.y[2] / 10) * (this.f.y[2] / 10)) * (-this.f.y[3] / 10);
  // カップの角度をPD制御
  var ru = 100 * (ra - this.cup1.body.GetAngle()) + 100 * (dra - this.cup1.body.GetAngularVelocity());
  this.cup1.rot.SetMotorSpeed(this.cup1.rot.motorSpeed + ru / 60);

  // ２つめのカップは角度の制御なし
  var u2 = 100 * (this.f.y[0] - this.cup2.body.GetPosition().x) + 100 * (this.f.y[1] - this.cup2.body.GetLinearVelocity().x);
  this.cup2.trans.SetMotorSpeed(this.cup2.trans.GetMotorSpeed() + u2 / 60);
  console.log(u2);
};


// 以下はLiquidFunに付属のtestbed.js

// shouldnt be a global :(
var particleColors = [
  new b2ParticleColor(0xff, 0x00, 0x00, 0xff), // red
  new b2ParticleColor(0x00, 0xff, 0x00, 0xff), // green
  new b2ParticleColor(0x00, 0x00, 0xff, 0xff), // blue
  new b2ParticleColor(0xff, 0x8c, 0x00, 0xff), // orange
  new b2ParticleColor(0x00, 0xce, 0xd1, 0xff), // turquoise
  new b2ParticleColor(0xff, 0x00, 0xff, 0xff), // magenta
  new b2ParticleColor(0xff, 0xd7, 0x00, 0xff), // gold
  new b2ParticleColor(0x00, 0xff, 0xff, 0xff) // cyan
];
var container;
var world = null;
var threeRenderer;
var renderer;
var camera;
var scene;
var objects = [];
var timeStep = 1.0 / 60.0;
var velocityIterations = 8;
var positionIterations = 3;
var test = {};
var projector = new THREE.Projector();
var planeZ = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
var g_groundBody = null;

var windowWidth = window.innerWidth;
var windowHeight = window.innerHeight;

//var GenerateOffsets = Module.cwrap("GenerateOffsets", 'null');

function initTestbed() {
  camera = new THREE.PerspectiveCamera(70, windowWidth / windowHeight, 1, 1000);
  threeRenderer = new THREE.WebGLRenderer();
  threeRenderer.setClearColor(0xFFFFFF);
  threeRenderer.setSize(windowWidth, windowHeight);

  camera.position.x = 0;
  camera.position.y = 0;
  camera.position.z = 100;
  scene = new THREE.Scene();
  camera.lookAt(scene.position);

  document.body.appendChild(this.threeRenderer.domElement);

  this.mouseJoint = null;

  // hack
  renderer = new Renderer();
  var gravity = new b2Vec2(0, -10);
  world = new b2World(gravity);
  Testbed();
}

function testSwitch(testName) {
  ResetWorld();
  world.SetGravity(new b2Vec2(0, -10));
  var bd = new b2BodyDef();
  g_groundBody = world.CreateBody(bd);
  test = new window[testName]();
}

function Testbed(obj) {
  // Init world
  //GenerateOffsets();
  //Init
  var that = this;
  document.addEventListener('keypress', function (event) {
    if (test.Keyboard !== undefined) {
      test.Keyboard(String.fromCharCode(event.which));
    }
  });
  document.addEventListener('keyup', function (event) {
    if (test.KeyboardUp !== undefined) {
      test.KeyboardUp(String.fromCharCode(event.which));
    }
  });

  document.addEventListener('mousedown', function (event) {
    var p = getMouseCoords(event);
    var aabb = new b2AABB();
    var d = new b2Vec2();

    d.Set(0.01, 0.01);
    b2Vec2.Sub(aabb.lowerBound, p, d);
    b2Vec2.Add(aabb.upperBound, p, d);

    var queryCallback = new QueryCallback(p);
    world.QueryAABB(queryCallback, aabb);

    if (queryCallback.fixture) {
      var body = queryCallback.fixture.body;
      var md = new b2MouseJointDef();
      md.bodyA = g_groundBody;
      md.bodyB = body;
      md.target = p;
      md.maxForce = 1000 * body.GetMass();
      that.mouseJoint = world.CreateJoint(md);
      body.SetAwake(true);
    }
    if (test.MouseDown !== undefined) {
      test.MouseDown(p);
    }

  });

  document.addEventListener('mousemove', function (event) {
    var p = getMouseCoords(event);
    if (that.mouseJoint) {
      that.mouseJoint.SetTarget(p);
    }
    if (test.MouseMove !== undefined) {
      test.MouseMove(p);
    }
  });

  document.addEventListener('mouseup', function (event) {
    if (that.mouseJoint) {
      world.DestroyJoint(that.mouseJoint);
      that.mouseJoint = null;
    }
    if (test.MouseUp !== undefined) {
      test.MouseUp(getMouseCoords(event));
    }
  });


  window.addEventListener('resize', onWindowResize, false);

  testSwitch("TestStabilizeLiquid");

  render();
}

var render = function () {
  // bring objects into world
  renderer.currentVertex = 0;
  if (test.Step !== undefined) {
    test.Step();
  } else {
    Step();
  }
  renderer.draw();

  threeRenderer.render(scene, camera);
  requestAnimationFrame(render);
};

var ResetWorld = function () {
  if (world !== null) {
    while (world.joints.length > 0) {
      world.DestroyJoint(world.joints[0]);
    }

    while (world.bodies.length > 0) {
      world.DestroyBody(world.bodies[0]);
    }

    while (world.particleSystems.length > 0) {
      world.DestroyParticleSystem(world.particleSystems[0]);
    }
  }
  camera.position.x = 0;
  camera.position.y = 0;
  camera.position.z = 100;
};

var Step = function () {
  world.Step(timeStep, velocityIterations, positionIterations);
};

/**@constructor*/
function QueryCallback(point) {
  this.point = point;
  this.fixture = null;
}

/**@return bool*/
QueryCallback.prototype.ReportFixture = function (fixture) {
  var body = fixture.body;
  if (body.GetType() === b2_dynamicBody) {
    var inside = fixture.TestPoint(this.point);
    if (inside) {
      this.fixture = fixture;
      return true;
    }
  }
  return false;
};

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  threeRenderer.setSize(window.innerWidth, window.innerHeight);
}

function getMouseCoords(event) {
  var mouse = new THREE.Vector3();
  mouse.x = (event.clientX / windowWidth) * 2 - 1;
  mouse.y = -(event.clientY / windowHeight) * 2 + 1;
  mouse.z = 0.5;

  projector.unprojectVector(mouse, camera);
  var dir = mouse.sub(camera.position).normalize();
  var distance = -camera.position.z / dir.z;
  var pos = camera.position.clone().add(dir.multiplyScalar(distance));
  var p = new b2Vec2(pos.x, pos.y);
  return p;
}


initTestbed();