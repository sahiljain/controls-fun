package ca.sahiljain;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Cursor;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.Box2D;
import com.badlogic.gdx.physics.box2d.Box2DDebugRenderer;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;

public class PhysicsTest extends ApplicationAdapter {
    private World world = new World(new Vector2(0, -10), true);
    private Box2DDebugRenderer renderer;
    private OrthographicCamera camera;
    private Body groundBody;
    private Body circleBody;
    private float circleStartX = 3.5f;
    private float reference = 4.8f;
    float prevError = reference - circleStartX;
    boolean firstIter = true;
    float errorSum = 0;
    private float kp = 0.4f;
    private float kd = 0.35f;
    private float ki = 0.00003f;

    float errorSumIn = 0;
    float prevErrorIn = 0;
    private float kpin = 400;
    private float kiin = 0.5f;
    private float kdin = 10;

    @Override
    public void create() {
        Box2D.init();
        camera = new OrthographicCamera(Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
        camera.position.set(camera.viewportWidth / 2, camera.viewportHeight / 2, 0);
//        System.out.println(reference);
        camera.update();

        renderer = new Box2DDebugRenderer();

        makeCircle();
        makeGround();
//        makeGuards();
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void makeGround() {
        BodyDef groundBodyDef = new BodyDef();
        groundBodyDef.type = BodyDef.BodyType.DynamicBody;
        groundBodyDef.position.set(new Vector2(ptm(camera.viewportWidth/2), ptm(100)));
        groundBody = world.createBody(groundBodyDef);
        PolygonShape groundBox = new PolygonShape();
        groundBox.setAsBox(ptm(camera.viewportWidth/2 - 30), ptm(5));
        groundBody.createFixture(groundBox, 1f);
        groundBox.dispose();

        BodyDef boxBodyDef = new BodyDef();
        boxBodyDef.type = BodyDef.BodyType.StaticBody;
        boxBodyDef.position.set(new Vector2(ptm(camera.viewportWidth/2), ptm(100)));
        Body boxBody = world.createBody(boxBodyDef);
        PolygonShape box = new PolygonShape();
        box.setAsBox(ptm(2), ptm(2));
        boxBody.createFixture(box, 0.0f);
        box.dispose();
        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = groundBody;
        jointDef.bodyB = boxBody;
        jointDef.localAnchorA.set(0,0);
        jointDef.collideConnected = false;
        world.createJoint(jointDef);
    }

    private void makeCircle() {
        BodyDef circleBodyDef = new BodyDef();
        circleBodyDef.type = BodyDef.BodyType.DynamicBody;
        circleBodyDef.position.set(circleStartX, 1.35f);
        circleBody = world.createBody(circleBodyDef);

        CircleShape circle = new CircleShape();
        circle.setRadius(ptm(20));

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = circle;
        fixtureDef.density = 2f;
        fixtureDef.friction = 0.2f;
        fixtureDef.restitution = 0.4f;

        circleBody.createFixture(fixtureDef);
        circle.dispose();
    }

    private void makeGuards() {
        BodyDef boxBodyDef = new BodyDef();
        boxBodyDef.type = BodyDef.BodyType.StaticBody;
        boxBodyDef.position.set(new Vector2(ptm(camera.viewportWidth/4), ptm(30)));
        Body boxBody = world.createBody(boxBodyDef);
        PolygonShape box = new PolygonShape();
        box.setAsBox(ptm(2), ptm(2));
        boxBody.createFixture(box, 0.0f);
        box.dispose();

        boxBodyDef = new BodyDef();
        boxBodyDef.type = BodyDef.BodyType.StaticBody;
        boxBodyDef.position.set(new Vector2(ptm(3*camera.viewportWidth/4), ptm(30)));
        boxBody = world.createBody(boxBodyDef);
        box = new PolygonShape();
        box.setAsBox(ptm(2), ptm(2));
        boxBody.createFixture(box, 0.0f);
        box.dispose();
    }

    @Override
    public void render() {
        Gdx.gl.glClearColor(0, 0, 0, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);
        world.step(Gdx.graphics.getDeltaTime(), 6, 2);
        float circleX = circleBody.getPosition().x;
        float error = reference - circleX;
        errorSum += error;

        float errorDelta = (error - prevError)/Gdx.graphics.getDeltaTime();

        float desiredAngle = -1 * (kp * error + kd * errorDelta + ki*errorSum);
        if (desiredAngle > Math.PI/6) desiredAngle = (float) (Math.PI/6);
        if (desiredAngle < -Math.PI/6) desiredAngle = (float) (-Math.PI/6);
        float curAngle = groundBody.getAngle();
        float errorIn = desiredAngle - curAngle;
        System.out.println(error);
        errorSumIn += errorIn;
        float errorInDelta = (errorIn - prevErrorIn)/Gdx.graphics.getDeltaTime();
        if (firstIter) {
            errorInDelta = 0f;
        }
        float torque = errorIn * kpin + errorSumIn * kiin + errorInDelta * kdin;
//        System.out.println(torque);
        if (torque > 100) torque = 100;
        if (torque < -100) torque = -100;
        groundBody.applyTorque(torque, true);

        prevError = error;
        prevErrorIn = errorIn;


        camera.update();
        renderer.render(world, camera.projection.translate(-(Gdx.graphics.getWidth() / 2), -(Gdx.graphics.getHeight() / 2), 0).scl((mtp(1f))));
        firstIter = false;
    }

    @Override
    public void dispose() {
    }

    static float scale(float in) {
        return in / 950 * Gdx.graphics.getWidth() * Gdx.graphics.getDensity();
    }

    static float ptm(float in) {
        return in / 50 / Gdx.graphics.getWidth() * 350;
    }

    static float mtp(float in) {
        return in * 50 * Gdx.graphics.getWidth() / 350;
    }

    static float rads(float in) {
        return (float) (in * 180 / Math.PI);
    }
}