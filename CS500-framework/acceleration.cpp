
#include "geom.h"
#include "raytrace.h"
#include "acceleration.h"

#include <bvh/sweep_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>
#include "realtime.h"

#include <random>
std::random_device device3;
std::mt19937_64 RNGen3(device3());
std::uniform_real_distribution<> myrandom3(0.0, 1.0);

/////////////////////////////
// Vector and ray conversions
RayInfo RayFromBvh(const bvh::Ray<float>& r)
{
    return RayInfo(vec3FromBvh(r.origin), vec3FromBvh(r.direction), myrandom3(RNGen3));
}
bvh::Ray<float> RayToBvh(const RayInfo& r)
{
    return bvh::Ray<float>(vec3ToBvh(r.origin), vec3ToBvh(r.direction));
}


/////////////////////////////
// SimpleBox
bvh::Vector3<float> vec3ToBvh(const vec3& v)
{
    return bvh::Vector3<float>(v[0], v[1], v[2]);
}

vec3 vec3FromBvh(const bvh::Vector3<float>& v)
{
    return vec3(v[0], v[1], v[2]);
}

SimpleBox::SimpleBox() : bvh::BoundingBox<float>() {}
SimpleBox::SimpleBox(const vec3 v) : bvh::BoundingBox<float>(vec3ToBvh(v)) {}

SimpleBox& SimpleBox::extend(const vec3 v)
{
    bvh::BoundingBox<float>::extend(vec3ToBvh(v));
    return *this;
}


/////////////////////////////
// BvhShape

BvhShape::BvhShape(Obj* s) : shape(s)
{
    boundingBox = new SimpleBox();
    boundingBox->min = vec3ToBvh(s->boxMin);
    //boundingBox->extend(s->boxMax);
    boundingBox->max = vec3ToBvh(s->boxMax);
}

SimpleBox BvhShape::bounding_box() const
{
    //  Return the shape's bounding box.
    return *boundingBox; // FIX - Done
}

bvh::Vector3<float> BvhShape::center() const
{
    return bounding_box().center();
}

std::optional<hitRecord> BvhShape::intersect(const bvh::Ray<float>& bvhray) const
{
    RayInfo ray = RayFromBvh(bvhray);
    hitRecord record;
    if (shape->Intersection(record, ray) == false)
        return std::nullopt;

    if (record.t < bvhray.tmin || record.t > bvhray.tmax)
        return std::nullopt;

    record.matKd = shape->material->Kd;
    record.matKs = shape->material->Ks;
    record.obj = shape;
    record.hit = true;

    if (shape->type == LIGHT)
        record.isLight = true;

    return record;  // FIX THIS (done)
}

AccelerationBvh::AccelerationBvh(std::vector<Obj*>& objs)
{
    // Wrap all Shape*'s with a bvh specific instance
    for (Obj* shape : objs) {
        shapeVector.emplace_back(shape);
    }

    // Magic found in the bvh examples:
    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(shapeVector.data(),
        shapeVector.size());
    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), shapeVector.size());

    bvh::SweepSahBuilder<bvh::Bvh<float>> builder(bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), shapeVector.size());
}

hitRecord AccelerationBvh::intersect(const RayInfo& ray)
{
    bvh::Ray<float> bvhRay = RayToBvh(ray);

    // Magic found in the bvh examples:
    bvh::ClosestPrimitiveIntersector<bvh::Bvh<float>, BvhShape> intersector(bvh, shapeVector.data());
    bvh::SingleRayTraverser<bvh::Bvh<float>> traverser(bvh);

    auto hit = traverser.traverse(bvhRay, intersector);
    if (hit) {
        return hit->intersection;
    }
	return  hitRecord();  // Return an IntersectionRecord which indicates NO-INTERSECTION
}