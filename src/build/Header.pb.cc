// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Header.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Header.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace gz_std_msgs {

namespace {

const ::google::protobuf::Descriptor* Header_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Header_reflection_ = NULL;
const ::google::protobuf::Descriptor* Header_Stamp_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Header_Stamp_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_Header_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_Header_2eproto() {
  protobuf_AddDesc_Header_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "Header.proto");
  GOOGLE_CHECK(file != NULL);
  Header_descriptor_ = file->message_type(0);
  static const int Header_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, frame_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, stamp_),
  };
  Header_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Header_descriptor_,
      Header::default_instance_,
      Header_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, _has_bits_[0]),
      -1,
      -1,
      sizeof(Header),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, _internal_metadata_),
      -1);
  Header_Stamp_descriptor_ = Header_descriptor_->nested_type(0);
  static const int Header_Stamp_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header_Stamp, sec_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header_Stamp, nsec_),
  };
  Header_Stamp_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Header_Stamp_descriptor_,
      Header_Stamp::default_instance_,
      Header_Stamp_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header_Stamp, _has_bits_[0]),
      -1,
      -1,
      sizeof(Header_Stamp),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header_Stamp, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_Header_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Header_descriptor_, &Header::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Header_Stamp_descriptor_, &Header_Stamp::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_Header_2eproto() {
  delete Header::default_instance_;
  delete Header_reflection_;
  delete Header_Stamp::default_instance_;
  delete Header_Stamp_reflection_;
}

void protobuf_AddDesc_Header_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_Header_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\014Header.proto\022\013gz_std_msgs\"h\n\006Header\022\020\n"
    "\010frame_id\030\001 \002(\t\022(\n\005stamp\030\002 \002(\0132\031.gz_std_"
    "msgs.Header.Stamp\032\"\n\005Stamp\022\013\n\003sec\030\001 \002(\r\022"
    "\014\n\004nsec\030\002 \002(\r", 133);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Header.proto", &protobuf_RegisterTypes);
  Header::default_instance_ = new Header();
  Header_Stamp::default_instance_ = new Header_Stamp();
  Header::default_instance_->InitAsDefaultInstance();
  Header_Stamp::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_Header_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_Header_2eproto {
  StaticDescriptorInitializer_Header_2eproto() {
    protobuf_AddDesc_Header_2eproto();
  }
} static_descriptor_initializer_Header_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Header_Stamp::kSecFieldNumber;
const int Header_Stamp::kNsecFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Header_Stamp::Header_Stamp()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:gz_std_msgs.Header.Stamp)
}

void Header_Stamp::InitAsDefaultInstance() {
}

Header_Stamp::Header_Stamp(const Header_Stamp& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:gz_std_msgs.Header.Stamp)
}

void Header_Stamp::SharedCtor() {
  _cached_size_ = 0;
  sec_ = 0u;
  nsec_ = 0u;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Header_Stamp::~Header_Stamp() {
  // @@protoc_insertion_point(destructor:gz_std_msgs.Header.Stamp)
  SharedDtor();
}

void Header_Stamp::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Header_Stamp::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Header_Stamp::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Header_Stamp_descriptor_;
}

const Header_Stamp& Header_Stamp::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Header_2eproto();
  return *default_instance_;
}

Header_Stamp* Header_Stamp::default_instance_ = NULL;

Header_Stamp* Header_Stamp::New(::google::protobuf::Arena* arena) const {
  Header_Stamp* n = new Header_Stamp;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Header_Stamp::Clear() {
// @@protoc_insertion_point(message_clear_start:gz_std_msgs.Header.Stamp)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(Header_Stamp, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<Header_Stamp*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  ZR_(sec_, nsec_);

#undef ZR_HELPER_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool Header_Stamp::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gz_std_msgs.Header.Stamp)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required uint32 sec = 1;
      case 1: {
        if (tag == 8) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &sec_)));
          set_has_sec();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_nsec;
        break;
      }

      // required uint32 nsec = 2;
      case 2: {
        if (tag == 16) {
         parse_nsec:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &nsec_)));
          set_has_nsec();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:gz_std_msgs.Header.Stamp)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gz_std_msgs.Header.Stamp)
  return false;
#undef DO_
}

void Header_Stamp::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gz_std_msgs.Header.Stamp)
  // required uint32 sec = 1;
  if (has_sec()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(1, this->sec(), output);
  }

  // required uint32 nsec = 2;
  if (has_nsec()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->nsec(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gz_std_msgs.Header.Stamp)
}

::google::protobuf::uint8* Header_Stamp::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:gz_std_msgs.Header.Stamp)
  // required uint32 sec = 1;
  if (has_sec()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(1, this->sec(), target);
  }

  // required uint32 nsec = 2;
  if (has_nsec()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->nsec(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gz_std_msgs.Header.Stamp)
  return target;
}

int Header_Stamp::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:gz_std_msgs.Header.Stamp)
  int total_size = 0;

  if (has_sec()) {
    // required uint32 sec = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->sec());
  }

  if (has_nsec()) {
    // required uint32 nsec = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->nsec());
  }

  return total_size;
}
int Header_Stamp::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:gz_std_msgs.Header.Stamp)
  int total_size = 0;

  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required uint32 sec = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->sec());

    // required uint32 nsec = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->nsec());

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Header_Stamp::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gz_std_msgs.Header.Stamp)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const Header_Stamp* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const Header_Stamp>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gz_std_msgs.Header.Stamp)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gz_std_msgs.Header.Stamp)
    MergeFrom(*source);
  }
}

void Header_Stamp::MergeFrom(const Header_Stamp& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gz_std_msgs.Header.Stamp)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_sec()) {
      set_sec(from.sec());
    }
    if (from.has_nsec()) {
      set_nsec(from.nsec());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void Header_Stamp::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gz_std_msgs.Header.Stamp)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Header_Stamp::CopyFrom(const Header_Stamp& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gz_std_msgs.Header.Stamp)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Header_Stamp::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;

  return true;
}

void Header_Stamp::Swap(Header_Stamp* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Header_Stamp::InternalSwap(Header_Stamp* other) {
  std::swap(sec_, other->sec_);
  std::swap(nsec_, other->nsec_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Header_Stamp::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Header_Stamp_descriptor_;
  metadata.reflection = Header_Stamp_reflection_;
  return metadata;
}


// -------------------------------------------------------------------

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Header::kFrameIdFieldNumber;
const int Header::kStampFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Header::Header()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:gz_std_msgs.Header)
}

void Header::InitAsDefaultInstance() {
  stamp_ = const_cast< ::gz_std_msgs::Header_Stamp*>(&::gz_std_msgs::Header_Stamp::default_instance());
}

Header::Header(const Header& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:gz_std_msgs.Header)
}

void Header::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  frame_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  stamp_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Header::~Header() {
  // @@protoc_insertion_point(destructor:gz_std_msgs.Header)
  SharedDtor();
}

void Header::SharedDtor() {
  frame_id_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != default_instance_) {
    delete stamp_;
  }
}

void Header::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Header::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Header_descriptor_;
}

const Header& Header::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Header_2eproto();
  return *default_instance_;
}

Header* Header::default_instance_ = NULL;

Header* Header::New(::google::protobuf::Arena* arena) const {
  Header* n = new Header;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Header::Clear() {
// @@protoc_insertion_point(message_clear_start:gz_std_msgs.Header)
  if (_has_bits_[0 / 32] & 3u) {
    if (has_frame_id()) {
      frame_id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
    if (has_stamp()) {
      if (stamp_ != NULL) stamp_->::gz_std_msgs::Header_Stamp::Clear();
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool Header::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gz_std_msgs.Header)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string frame_id = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_frame_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->frame_id().data(), this->frame_id().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "gz_std_msgs.Header.frame_id");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_stamp;
        break;
      }

      // required .gz_std_msgs.Header.Stamp stamp = 2;
      case 2: {
        if (tag == 18) {
         parse_stamp:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_stamp()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:gz_std_msgs.Header)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gz_std_msgs.Header)
  return false;
#undef DO_
}

void Header::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gz_std_msgs.Header)
  // required string frame_id = 1;
  if (has_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), this->frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gz_std_msgs.Header.frame_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->frame_id(), output);
  }

  // required .gz_std_msgs.Header.Stamp stamp = 2;
  if (has_stamp()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, *this->stamp_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gz_std_msgs.Header)
}

::google::protobuf::uint8* Header::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:gz_std_msgs.Header)
  // required string frame_id = 1;
  if (has_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), this->frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gz_std_msgs.Header.frame_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->frame_id(), target);
  }

  // required .gz_std_msgs.Header.Stamp stamp = 2;
  if (has_stamp()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        2, *this->stamp_, false, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gz_std_msgs.Header)
  return target;
}

int Header::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:gz_std_msgs.Header)
  int total_size = 0;

  if (has_frame_id()) {
    // required string frame_id = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->frame_id());
  }

  if (has_stamp()) {
    // required .gz_std_msgs.Header.Stamp stamp = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->stamp_);
  }

  return total_size;
}
int Header::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:gz_std_msgs.Header)
  int total_size = 0;

  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required string frame_id = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->frame_id());

    // required .gz_std_msgs.Header.Stamp stamp = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->stamp_);

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Header::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gz_std_msgs.Header)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const Header* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const Header>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gz_std_msgs.Header)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gz_std_msgs.Header)
    MergeFrom(*source);
  }
}

void Header::MergeFrom(const Header& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gz_std_msgs.Header)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_frame_id()) {
      set_has_frame_id();
      frame_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
    }
    if (from.has_stamp()) {
      mutable_stamp()->::gz_std_msgs::Header_Stamp::MergeFrom(from.stamp());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void Header::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gz_std_msgs.Header)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Header::CopyFrom(const Header& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gz_std_msgs.Header)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Header::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;

  if (has_stamp()) {
    if (!this->stamp_->IsInitialized()) return false;
  }
  return true;
}

void Header::Swap(Header* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Header::InternalSwap(Header* other) {
  frame_id_.Swap(&other->frame_id_);
  std::swap(stamp_, other->stamp_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Header::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Header_descriptor_;
  metadata.reflection = Header_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Header_Stamp

// required uint32 sec = 1;
bool Header_Stamp::has_sec() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void Header_Stamp::set_has_sec() {
  _has_bits_[0] |= 0x00000001u;
}
void Header_Stamp::clear_has_sec() {
  _has_bits_[0] &= ~0x00000001u;
}
void Header_Stamp::clear_sec() {
  sec_ = 0u;
  clear_has_sec();
}
 ::google::protobuf::uint32 Header_Stamp::sec() const {
  // @@protoc_insertion_point(field_get:gz_std_msgs.Header.Stamp.sec)
  return sec_;
}
 void Header_Stamp::set_sec(::google::protobuf::uint32 value) {
  set_has_sec();
  sec_ = value;
  // @@protoc_insertion_point(field_set:gz_std_msgs.Header.Stamp.sec)
}

// required uint32 nsec = 2;
bool Header_Stamp::has_nsec() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void Header_Stamp::set_has_nsec() {
  _has_bits_[0] |= 0x00000002u;
}
void Header_Stamp::clear_has_nsec() {
  _has_bits_[0] &= ~0x00000002u;
}
void Header_Stamp::clear_nsec() {
  nsec_ = 0u;
  clear_has_nsec();
}
 ::google::protobuf::uint32 Header_Stamp::nsec() const {
  // @@protoc_insertion_point(field_get:gz_std_msgs.Header.Stamp.nsec)
  return nsec_;
}
 void Header_Stamp::set_nsec(::google::protobuf::uint32 value) {
  set_has_nsec();
  nsec_ = value;
  // @@protoc_insertion_point(field_set:gz_std_msgs.Header.Stamp.nsec)
}

// -------------------------------------------------------------------

// Header

// required string frame_id = 1;
bool Header::has_frame_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void Header::set_has_frame_id() {
  _has_bits_[0] |= 0x00000001u;
}
void Header::clear_has_frame_id() {
  _has_bits_[0] &= ~0x00000001u;
}
void Header::clear_frame_id() {
  frame_id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_frame_id();
}
 const ::std::string& Header::frame_id() const {
  // @@protoc_insertion_point(field_get:gz_std_msgs.Header.frame_id)
  return frame_id_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Header::set_frame_id(const ::std::string& value) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gz_std_msgs.Header.frame_id)
}
 void Header::set_frame_id(const char* value) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gz_std_msgs.Header.frame_id)
}
 void Header::set_frame_id(const char* value, size_t size) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gz_std_msgs.Header.frame_id)
}
 ::std::string* Header::mutable_frame_id() {
  set_has_frame_id();
  // @@protoc_insertion_point(field_mutable:gz_std_msgs.Header.frame_id)
  return frame_id_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* Header::release_frame_id() {
  // @@protoc_insertion_point(field_release:gz_std_msgs.Header.frame_id)
  clear_has_frame_id();
  return frame_id_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void Header::set_allocated_frame_id(::std::string* frame_id) {
  if (frame_id != NULL) {
    set_has_frame_id();
  } else {
    clear_has_frame_id();
  }
  frame_id_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), frame_id);
  // @@protoc_insertion_point(field_set_allocated:gz_std_msgs.Header.frame_id)
}

// required .gz_std_msgs.Header.Stamp stamp = 2;
bool Header::has_stamp() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void Header::set_has_stamp() {
  _has_bits_[0] |= 0x00000002u;
}
void Header::clear_has_stamp() {
  _has_bits_[0] &= ~0x00000002u;
}
void Header::clear_stamp() {
  if (stamp_ != NULL) stamp_->::gz_std_msgs::Header_Stamp::Clear();
  clear_has_stamp();
}
const ::gz_std_msgs::Header_Stamp& Header::stamp() const {
  // @@protoc_insertion_point(field_get:gz_std_msgs.Header.stamp)
  return stamp_ != NULL ? *stamp_ : *default_instance_->stamp_;
}
::gz_std_msgs::Header_Stamp* Header::mutable_stamp() {
  set_has_stamp();
  if (stamp_ == NULL) {
    stamp_ = new ::gz_std_msgs::Header_Stamp;
  }
  // @@protoc_insertion_point(field_mutable:gz_std_msgs.Header.stamp)
  return stamp_;
}
::gz_std_msgs::Header_Stamp* Header::release_stamp() {
  // @@protoc_insertion_point(field_release:gz_std_msgs.Header.stamp)
  clear_has_stamp();
  ::gz_std_msgs::Header_Stamp* temp = stamp_;
  stamp_ = NULL;
  return temp;
}
void Header::set_allocated_stamp(::gz_std_msgs::Header_Stamp* stamp) {
  delete stamp_;
  stamp_ = stamp;
  if (stamp) {
    set_has_stamp();
  } else {
    clear_has_stamp();
  }
  // @@protoc_insertion_point(field_set_allocated:gz_std_msgs.Header.stamp)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace gz_std_msgs

// @@protoc_insertion_point(global_scope)
