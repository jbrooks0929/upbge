/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_curves.hh"

namespace blender::nodes {

/* --------------------------------------------------------------------
 * Spline Length
 */

static VArray<float> construct_spline_length_gvarray(const CurveComponent &component,
                                                     const AttributeDomain domain)
{
  if (!component.has_curves()) {
    return {};
  }
  const Curves &curves_id = *component.get_for_read();
  const bke::CurvesGeometry &curves = bke::CurvesGeometry::wrap(curves_id.geometry);

  curves.ensure_evaluated_lengths();

  VArray<bool> cyclic = curves.cyclic();
  VArray<float> lengths = VArray<float>::ForFunc(
      curves.curves_num(), [&curves, cyclic = std::move(cyclic)](int64_t index) {
        return curves.evaluated_length_total_for_curve(index, cyclic[index]);
      });

  if (domain == ATTR_DOMAIN_CURVE) {
    return lengths;
  }

  if (domain == ATTR_DOMAIN_POINT) {
    return component.attribute_try_adapt_domain<float>(
        std::move(lengths), ATTR_DOMAIN_CURVE, ATTR_DOMAIN_POINT);
  }

  return {};
}

SplineLengthFieldInput::SplineLengthFieldInput()
    : GeometryFieldInput(CPPType::get<float>(), "Spline Length node")
{
  category_ = Category::Generated;
}

GVArray SplineLengthFieldInput::get_varray_for_context(const GeometryComponent &component,
                                                       const AttributeDomain domain,
                                                       IndexMask UNUSED(mask)) const
{
  if (component.type() == GEO_COMPONENT_TYPE_CURVE) {
    const CurveComponent &curve_component = static_cast<const CurveComponent &>(component);
    return construct_spline_length_gvarray(curve_component, domain);
  }
  return {};
}

uint64_t SplineLengthFieldInput::hash() const
{
  /* Some random constant hash. */
  return 3549623580;
}

bool SplineLengthFieldInput::is_equal_to(const fn::FieldNode &other) const
{
  return dynamic_cast<const SplineLengthFieldInput *>(&other) != nullptr;
}

}  // namespace blender::nodes

namespace blender::nodes::node_geo_input_spline_length_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Float>(N_("Length")).field_source();
  b.add_output<decl::Int>(N_("Point Count")).field_source();
}

/* --------------------------------------------------------------------
 * Spline Count
 */

static VArray<int> construct_spline_count_gvarray(const CurveComponent &component,
                                                  const AttributeDomain domain)
{
  if (!component.has_curves()) {
    return {};
  }
  const Curves &curves_id = *component.get_for_read();
  const bke::CurvesGeometry &curves = bke::CurvesGeometry::wrap(curves_id.geometry);

  auto count_fn = [curves](int64_t i) { return curves.points_for_curve(i).size(); };

  if (domain == ATTR_DOMAIN_CURVE) {
    return VArray<int>::ForFunc(curves.curves_num(), count_fn);
  }
  if (domain == ATTR_DOMAIN_POINT) {
    VArray<int> count = VArray<int>::ForFunc(curves.curves_num(), count_fn);
    return component.attribute_try_adapt_domain<int>(
        std::move(count), ATTR_DOMAIN_CURVE, ATTR_DOMAIN_POINT);
  }

  return {};
}

class SplineCountFieldInput final : public GeometryFieldInput {
 public:
  SplineCountFieldInput() : GeometryFieldInput(CPPType::get<int>(), "Spline Point Count")
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const GeometryComponent &component,
                                 const AttributeDomain domain,
                                 IndexMask UNUSED(mask)) const final
  {
    if (component.type() == GEO_COMPONENT_TYPE_CURVE) {
      const CurveComponent &curve_component = static_cast<const CurveComponent &>(component);
      return construct_spline_count_gvarray(curve_component, domain);
    }
    return {};
  }

  uint64_t hash() const override
  {
    /* Some random constant hash. */
    return 456364322625;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    return dynamic_cast<const SplineCountFieldInput *>(&other) != nullptr;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<float> spline_length_field{std::make_shared<SplineLengthFieldInput>()};
  Field<int> spline_count_field{std::make_shared<SplineCountFieldInput>()};

  params.set_output("Length", std::move(spline_length_field));
  params.set_output("Point Count", std::move(spline_count_field));
}

}  // namespace blender::nodes::node_geo_input_spline_length_cc

void register_node_type_geo_input_spline_length()
{
  namespace file_ns = blender::nodes::node_geo_input_spline_length_cc;

  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_INPUT_SPLINE_LENGTH, "Spline Length", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
